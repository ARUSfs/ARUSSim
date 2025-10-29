#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <array>
#include <cstdint>
#include <array>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring> 
#include "std_msgs/msg/header.hpp"
#include "arussim_msgs/msg/cmd.hpp"
#include "arussim_msgs/msg/cmd4_wd.hpp"

#include "std_msgs/msg/float32.hpp"

class ControlSim : public rclcpp::Node
{
    public:
        ControlSim();
        

 private:
    void receive_can();
    void send_torque();
    void send_state();
    int can_socket_;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
    struct can_frame frame;
    rclcpp::TimerBase::SharedPtr timer_receive_;
    rclcpp::TimerBase::SharedPtr timer_send_;
    rclcpp::TimerBase::SharedPtr timer_state_;



    double kMass;
    float kRdyn;
    float kGearRatio;

    int16_t torque_i_;
    int16_t vx_i_;
    int16_t vy_i_;
    int16_t r_i_;
    float acc_;
    float target_r_;
    float vx_;
    float vy_;
    float r_;
    int16_t acc_scaled_;
    int16_t yaw_scaled_;
    int16_t vx_scaled_;
    int16_t vy_scaled_;
    int16_t r_scaled_;

    rclcpp::Publisher<arussim_msgs::msg::Cmd4WD>::SharedPtr torque_pub_;
    rclcpp::Publisher<arussim_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_vx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_vy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_r_pub_;
};