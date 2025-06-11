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
#include "PointXYZProbColorScore.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "arussim/csv_generator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "arussim/sensors.hpp"
#include <random>
#include <nlohmann/json.hpp>
#include "std_msgs/msg/string.hpp"

#include "controller_sim/controller_sim.hpp"
#include "controller_sim/estimation.hpp"
#include "controller_sim/power_limitation.hpp"
#include "controller_sim/traction_control.hpp"
#include "controller_sim/torque_vectoring.hpp"

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
    ControllerSim controller_sim_;

    std::string kTrackName;
    double kStateUpdateRate;
    double kControllerRate; 
    double kWheelBase;
    double kFOV;
    double kPosLidarX;
    double kSensorRate;
    double kNoisePerception;
    double kNoiseColor;
    double kMinPerceptionX;
    double kSimulationSpeedMultiplier;
    bool kTorqueVectoring;
    bool kDebug;
    
    //Car boundaries
    double kCOGFrontDist;
    double kCOGBackDist;
    double kCarWidth;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_last_cmd_;
    double input_acc_;
    double input_delta_;
    double target_r_;
    std::vector<double> torque_cmd_;

    visualization_msgs::msg::Marker marker_;
    pcl::PointCloud<ConeXYZColorScore> track_;
    arussim_msgs::msg::Trajectory fixed_trajectory_msg_;
    std::string prev_circuit_;
    std::string track_name_;

    std::vector<std::pair<double, double>> tpl_cones_;

    bool use_tpl_ = false;
    bool started_acc_ = false;

    double tpl_coef_a_ = 0;
    double tpl_coef_b_ = 0;

    double prev_dist_to_tpl_ = 0;

    double mid_tpl_x_ = 0;
    double mid_tpl_y_ = 0;

    bool kCSVVehicleDynamics;
    std::shared_ptr<CSVGenerator> csv_generator_vehicle_dynamics_;

    bool kCSVState;
    std::shared_ptr<CSVGenerator> csv_generator_state_;

    visualization_msgs::msg::MarkerArray current_cone_markers_;

    /**
     * @brief Callback function for the slow timer.
     * 
     * This method is called at regular intervals to update and publish sensor data, 
     * such as the track and perception point clouds.
     */
    void on_slow_timer();

    /**
     * @brief Callback function for the controller timer.
     * 
     * This method is called at regular intervals to update and publish low level controller
     * action, simulating vehicle control unit from real car.
     */
    void on_controller_sim_timer();

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
     * @param track_msg Mensaje que contiene el nombre del track.
     */
    void load_track(const std_msgs::msg::String::SharedPtr track_msg);

    /**
     * @brief Detects if the vehicle is between two TPLs.
     */
    void check_lap();

    /**
     * @brief Checks if car has started acc event
     * 
     */
    void check_acc_start();

    /**
     * @brief Make a MarkerArray of all cones of the track
     * 
     */
    void cone_visualization();

    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr controller_sim_timer_;
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr circuit_sub_;
};