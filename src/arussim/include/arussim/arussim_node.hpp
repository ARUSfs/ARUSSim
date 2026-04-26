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

#include <linux/can.h>       
#include <linux/can/raw.h>   
#include <net/if.h>          
#include <sys/ioctl.h>       
#include <sys/socket.h>    
#include <fcntl.h>
#include <thread> 

#include "control.h"

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
    std::string kSimulationCar;
    double kStateUpdateRate;
    double kControllerRate; 
    std::string kSimulationMode;
    bool kUseGSS;
    double kWheelBase;
    double kLidarFOV;
    double kMinLidarDistance;
    double kMaxLidarDistance;
    double kCameraFOV;
    double kPosLidarX;
    double kPosCameraX;
    double kCameraColorProbability;
    double kSensorRate;
    double kNoiseLidarPosPerception;
    double kNoiseLidarProbPerception;
    double kNoiseLidarColor;
    double kNoiseCameraPerception;
    double kNoiseCameraColor;
    double kMinPerceptionX;
    double kSimulationSpeedMultiplier;
    bool kDebug;
    double kGearRatio = 12.48;
    
    // Sensor data with noise
    double noisy_ax_ = 0.0;
    double noisy_ay_ = 0.0;
    double noisy_r_ = 0.0;
    double noisy_ws_fl_ = 0.0;
    double noisy_ws_fr_ = 0.0;
    double noisy_ws_rl_ = 0.0;
    double noisy_ws_rr_ = 0.0;
    double noisy_vx_ = 0.0;
    double noisy_vy_ = 0.0;
    double noisy_delta_ = 0.0;

    //Car boundaries
    double kCOGFrontDist;
    double kCOGBackDist;
    double kCarWidth;

    // Car parameters
    std::string simulation_car_csv_;
    std::map<std::string, double> parameters_map_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_last_cmd_;
    double input_acc_;
    double input_delta_;
    double target_r_;
    std::vector<double> torque_cmd_;

    //Can
    float can_acc_;
    float can_target_r_;
    float can_delta_;
    std::vector<double> can_torque_cmd_;
    uint16_t as_status_ = 0x02;

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
    * @brief Configures and links POSIX sockets for the CAN bus
    */
    void init_can_sockets();
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
     * @brief Callback functions for receiving noisy sensor data.
     * 
     */
    void noisy_ax_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void noisy_ay_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void noisy_r_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void noisy_ws_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg);
    void noisy_vx_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void noisy_vy_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void noisy_delta_callback(const std_msgs::msg::Float32::SharedPtr msg);

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
     * @brief Callback for receiving EBS (Emergency Brake System) commands.
     * 
     * This method processes EBS commands to apply emergency braking if necessary.
     * 
     * @param msg The EBS command message.
     */
    void ebs_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    /**
     * @brief Callback for receiving reset commands.
     * 
     * This method resets the vehicle's state to the initial pose.
     * 
     * @param msg The reset command message.
     */
    void reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback for receiving launch commands.
     * 
     * This method is needed for the HIL control, to start the simulation when the real Raspberry Pi starts sending commands.
     * 
     */
    void launch_callback(const std_msgs::msg::Bool::SharedPtr msg);

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
     * @brief Loads the car parameters from resources.
     * 
     */
    std::string select_csv(const std::string& kSimulationCar);

    /**
     * @brief Resolves the CSV file path based on the simulation_car 
     */
    std::string get_csv_path(const std::string& csv_filename);

    /**
     * @brief Load parameters from de according CSV file
     */
    std::map<std::string, double> load_car_parameters(const std::string &filepath);

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

    /**
     * @brief Can reception functions for both CAN interfaces
     */
    void receive_can_0();
    void receive_can_1();
    void receive_can_2();


    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_ax_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_ay_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_r_sub_;
    rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr noisy_ws_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_vx_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_vy_sub_;   
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr noisy_delta_sub_;   
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr controller_sim_timer_;
    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ebs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_telep_sub_;

    rclcpp::Publisher<arussim_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr slip_ratio_pub_;
    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr slip_angle_pub_;
    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr tire_load_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_vx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_vy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_r_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr track_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_perception_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_perception_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_signal_pub_;
    rclcpp::Publisher<arussim_msgs::msg::PointXY>::SharedPtr hit_cones_pub_;
    rclcpp::Publisher<arussim_msgs::msg::Trajectory>::SharedPtr fixed_trajectory_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr circuit_sub_;

    //CAN Communication
    int can_socket_0_;
    struct ifreq ifr_0_{};
    struct sockaddr_can addr_0_{};
    struct can_frame frame_0_;
    std::thread thread_0_;

    int can_socket_1_;
    struct ifreq ifr_1_{};
    struct sockaddr_can addr_1_{};
    struct can_frame frame_1_;
    std::thread thread_1_;

    int can_socket_2_;
    struct ifreq ifr_2_{};
    struct sockaddr_can addr_2_{};
    struct can_frame frame_2_;
    std::thread thread_2_;
    
};