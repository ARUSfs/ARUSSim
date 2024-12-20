/**
 * @file sensors.hpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Header file for sensors.cpp.
 * @version 0.1
 * @date 2024-10-16 
 */
#include <rclcpp/rclcpp.hpp>
#include "arussim_msgs/msg/state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "std_msgs/msg/float32.hpp"
#include <random>

/**
 * @class Sensors
 * @brief Sensors class that declares variables
 */
class Sensors : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Sensors object
     * 
     */
    Sensors(); // Constructor

private:
    // Variables
    double x_ = 0;
    double y_ = 0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double r_ = 0;
    double ax_ = 0;
    double ay_ = 0;
    double delta_ = 0;
    arussim_msgs::msg::FourWheelDrive wheel_speed;
    arussim_msgs::msg::FourWheelDrive torque_cmd;

    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } wheel_speed_;

    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } torque_cmd_;

    double kExtensometerFrequency;
    double kNoiseExtensometer;

    double kWheelSpeedFrequency;
    double kNoiseWheelSpeedFrontRight;
    double kNoiseWheelSpeedFrontLeft;
    double kNoiseWheelSpeedRearRight;
    double kNoiseWheelSpeedRearLeft;

    double kTorqueFrequency;
    double kNoiseTorqueFrontRight;
    double kNoiseTorqueFrontLeft;
    double kNoiseTorqueRearRight;
    double kNoiseTorqueRearLeft;

    double kImuFrequency;
    double kNoiseImuX;
    double kNoiseImuY;
    double kNoiseImuYaw;
    double kNoiseImuAx;
    double kNoiseImuAy;
    double kNoiseImuR;

    /**
     * @brief Callback function for the state subscriber
     * 
     * @param msg 
     */
    void state_callback(const arussim_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Timer function for the extensometer
     * 
     */
    void extensometer_timer();

    /**
     * @brief Timer function for the wheel speed sensors
     * 
     */
    void wheel_speed_timer();
    
    /**
     * @brief Timer function for the IMU
     * 
     */
    void imu_timer();

    /**
     * @brief Timer function for the 4WD torque
     * 
     */
    void torque_cmd_timer();
    
    
    // ROS Communication
    rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_; // State subscriber

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // IMU publisher
    rclcpp::TimerBase::SharedPtr imu_timer_; // IMU timer

    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr ws_pub_; // Wheel speed publisher
    rclcpp::TimerBase::SharedPtr ws_timer_; // Wheel speed timer

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ext_pub_; // Extensometer publisher
    rclcpp::TimerBase::SharedPtr ext_timer_; // Extensometer timer

    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr torque_pub_; // Torque publisher
    rclcpp::TimerBase::SharedPtr torque_timer_; // Torque timer
};