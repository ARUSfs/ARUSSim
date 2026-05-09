/**
 * @file sensors.hpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Header file for sensors.cpp.
 * @version 0.1
 * @date 2024-10-16 
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "arussim_msgs/msg/state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "std_msgs/msg/float32.hpp"
#include <map>
#include <random>
#include <string>
#include <vector>
#include <linux/can.h>       
#include <linux/can/raw.h>   
#include <net/if.h>          
#include <sys/ioctl.h>       
#include <sys/socket.h>  

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

    //Define CAN frame and signal structures
    enum class CanBus {
        kCan0,
        kCan1,
        kCan2
    };

    struct CanSignal {
        int bit_in;
        int bit_fin;
        bool is_signed;
        double scale;
        double offset;
    };
    struct CanFrame {
        uint32_t id;
        int size;
        std::map<std::string, CanSignal> signals;
        CanBus can_bus = CanBus::kCan0;
    };
    static std::vector<CanFrame> frames; 

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
    arussim_msgs::msg::FourWheelDrive wheel_speed_msg_;
    arussim_msgs::msg::FourWheelDrive torque_cmd_msg_;

    double as_status_ = 2.0;

    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } motor_speed_;

    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } torque_cmd_;

    std::string kSimulationMode;
    double kGssFrequency;
    double kNoiseGssVx;
    double kNoiseGssVy;
    double kNoiseGssAx;
    double kNoiseGssAy;
    double kNoiseGssR;

    double kImuFrequency;
    double kNoiseImuX;
    double kNoiseImuY;
    double kNoiseImuAx;
    double kNoiseImuAy;
    double kNoiseImuR;
    double kNoiseImuPose;

    double kInverterFrequency;
    double kNoiseMotorSpeedFrontRight;
    double kNoiseMotorSpeedFrontLeft;
    double kNoiseMotorSpeedRearRight;
    double kNoiseMotorSpeedRearLeft;
    double kNoiseTorqueFrontRight;
    double kNoiseTorqueFrontLeft;
    double kNoiseTorqueRearRight;
    double kNoiseTorqueRearLeft;
    double kGearRatio = 12.48;

    double kExtensometerFrequency;
    double kNoiseExtensometer;

    double kBMSFrequency;
    double kNoiseBatteryVoltage;

    double kASFrequency;

    /**
     * @brief Callback for receiving reset commands.
     * 
     * This method resets the as_status_ = 0x02.
     * 
     * @param msg The reset command message.
     */
    void reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback for receiving launch commands.
     * 
     * This method sets the as_status_ = 0x01, which indicates that the vehicle is launched.
     * 
     */
    void launch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback function for the state subscriber
     * 
     * @param msg 
     */
    void state_callback(const arussim_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Timer function for the groundspeed sensor
     * 
     */
    void groundspeed_timer();
    
    /**
     * @brief Timer function for the IMU
     * 
     */
    void imu_timer();

    /**
     * @brief Timer function for the inverters
     * 
     */
    void inverter_timer();
    
    /**
     * @brief Timer function for the BMS
     * 
     */
    void bms_timer();
    
    /**
     * @brief Timer function for the extensometer
     * 
     */
    void extensometer_timer();

    /**
     * @brief Timer function for the AS PCB
     */
    void as_timer();

    static uint64_t encode_signal(double value, double scale, double offset, int bit_len, bool is_signed);
    void send_can_frame(const CanFrame &frame, const std::map<std::string,double> &values);
    int setup_can_socket(const char * interface_name);

    // ROS Communication
    rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_; // State subscriber

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gss_vx_pub_; // Groundspeed publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gss_vy_pub_; // Groundspeed publisher
    rclcpp::TimerBase::SharedPtr gss_timer_; // Groundspeed timer

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ax_pub_; // ax publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ay_pub_; // ay publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr r_pub_; // r publisher
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr x_pub_; // x publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr y_pub_; // y publisher
    rclcpp::TimerBase::SharedPtr imu_timer_; // IMU timer

    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr motor_speed_pub_; // Motor speed publisher
    rclcpp::Publisher<arussim_msgs::msg::FourWheelDrive>::SharedPtr torque_pub_; // Torque publisher
    rclcpp::TimerBase::SharedPtr inv_timer_; // Wheel speed timer

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ext_pub_; // Extensometer publisher
    rclcpp::TimerBase::SharedPtr ext_timer_; // Extensometer timer
    
    rclcpp::TimerBase::SharedPtr bms_timer_; // BMS timer

    rclcpp::TimerBase::SharedPtr as_timer_; // AS PCB timer

    //CAN Communication
    int can0_socket_ = -1;
    int can1_socket_ = -1;
    int can2_socket_ = -1;
    struct can_frame canMsg_{}; 
};