/**
 * @file arussim_node.hpp
 * @brief Header file for the ARUS Team's simulator node (ARUSim).
 * 
 * This file declares the Simulator class, which simulates the behavior of a vehicle 
 * in a racing environment, including sensor data generation, state updates, and 
 * visualization in RViz.
 */

#include <rclcpp/rclcpp.hpp>

#include "arussim_msgs/msg/state.hpp"
#include "arussim_msgs/msg/cmd.hpp"
#include "arussim_msgs/msg/trajectory.hpp"
#include "arussim_msgs/msg/point_xy.hpp"
#include "arussim_msgs/srv/set_fov.hpp"

#include "arussim/vehicle_dynamics.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/bool.hpp"
#include <set>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "ConeXYZColorScore.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @class Simulator
 * @brief The Simulator class is responsible for simulating the vehicle dynamics, 
 * sensor data, and managing communication with ROS nodes.
 * 
 * The Simulator class handles the simulation of a vehicle's position, orientation, 
 * and velocity, along with generating and broadcasting sensor data. It also publishes 
 * visualization data to RViz and listens for control commands.
 */

class Simulator : public rclcpp::Node
{
  public:
  /**
     * @brief Constructor for the Simulator class.
     * 
     * This initializes the Simulator node, declares ROS parameters, sets up publishers, 
     * subscribers, and timers, and loads necessary resources like the track and vehicle mesh.
     */
    Simulator();

  private:
    VehicleDynamics vehicle_dynamics_;

    std::string kTrackName;
    double kFrictionCoef;
    double kStateUpdateRate;
    double kMass;
    double kWheelBase;
    double kFOV;
    double kPosLidarX;
    double kSensorRate;
    double kNoisePerception;
    double kMinPerceptionX;

    //Car boundaries
    double kCOGFrontDist;
    double kCOGBackDist;
    double kCarWidth;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_last_cmd_;
    double input_acc_;
    double input_delta_;

    visualization_msgs::msg::Marker marker_;
    pcl::PointCloud<ConeXYZColorScore> track_;
    arussim_msgs::msg::Trajectory fixed_trajectory_msg_;


    std::vector<std::pair<double, double>> tpl_cones_;

    bool use_tpl_ = false;

    double tpl_coef_a_ = 0;
    double tpl_coef_b_ = 0;

    double prev_dist_to_tpl_ = 0;

    double mid_tpl_x_ = 0;
    double mid_tpl_y_ = 0;
    
    /**
     * @brief Callback function for the slow timer.
     * 
     * This method is called at regular intervals to update and publish sensor data, 
     * such as the track and perception point clouds.
     */
    void on_slow_timer();

    /**
     * @brief Callback function for the fast timer.
     * 
     * This method is called at regular intervals to update the vehicle's state and 
     * broadcast its transform to the ROS TF system.
     */
    void on_fast_timer();

    /**
     * @brief Callback for receiving control commands.
     * 
     * This method processes incoming control commands (acceleration and steering angle) 
     * to update the vehicle's dynamics.
     * 
     * @param msg The control command message.
     */
    void cmd_callback(const arussim_msgs::msg::Cmd::SharedPtr msg);
    
    /**
     * @brief Callback for receiving teleportation commands from RViz.
     * 
     * This method teleports the vehicle to a new pose as commanded via RViz.
     * 
     * @param msg The pose message received from RViz.
     */
    void rviz_telep_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    /**
     * @brief Callback for receiving reset commands.
     * 
     * This method resets the vehicle's state to the initial pose.
     * 
     * @param msg The reset command message.
     */
    void reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Broadcasts the vehicle's current pose to the ROS TF system.
     * 
     * This method sends the vehicle's transform to the TF tree so that other nodes 
     * can track its position and orientation.
     */
    void broadcast_transform();

    /**
     * @brief Loads the track and the fixed path from resources.
     * 
     * @param track 
     */
    void load_track(const pcl::PointCloud<ConeXYZColorScore>& track);

    /**
     * @brief Detects if the vehicle is between two TPLs.
     */
    void check_lap();

    /**
     * @brief Make a MarkerArray of all cones of the track
     * 
     */
    void cone_visualization();

    /**
     * @brief Service handler for setting the FOV.
     * 
     * This method updates the FOV parameter based on a service request.
     * 
     * @param request The service request message.
     * @param response The service response message.
     */
    void handle_set_fov(
      const std::shared_ptr<arussim_msgs::srv::SetFOV::Request> request,
      std::shared_ptr<arussim_msgs::srv::SetFOV::Response> response);


    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_telep_sub_;
    rclcpp::Publisher<arussim_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr track_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr perception_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_signal_pub_;
    rclcpp::Publisher<arussim_msgs::msg::PointXY>::SharedPtr hit_cones_pub_;
    rclcpp::Publisher<arussim_msgs::msg::Trajectory>::SharedPtr fixed_trajectory_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Service<arussim_msgs::srv::SetFOV>::SharedPtr set_fov_service_;
};