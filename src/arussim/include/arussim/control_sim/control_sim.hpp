#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <array>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring> 
#include "std_msgs/msg/bool.hpp"

#include "arussim/control_sim/SensorData.h"
#include "arussim/control_sim/Parameters.h"
#include "arussim/control_sim/aux_functions.h"
#include "arussim/control_sim/estimation.h"
#include "arussim/control_sim/torque_vectoring.h"
#include "arussim/control_sim/traction_control.h"
#include "arussim/control_sim/power_control.h"


class ControlSim : public rclcpp::Node
{
    public:
        ControlSim();
        

    private:
        SensorData sensors;
        Parameters parameters;
        DV dv;
        TIRE tire;
        PID pid;

        void Parameters_Init(Parameters *);
        void receive_can();
        void send_torque(float torque_out[4]);
        void send_state();
        void default_task();
        int can_socket_;
        struct ifreq ifr_;
        struct sockaddr_can addr_;
        struct can_frame frame;
        rclcpp::TimerBase::SharedPtr timer_receive_;
        rclcpp::TimerBase::SharedPtr timer_defaultTask_;
        rclcpp::Clock::SharedPtr clock_;
        std::thread thread_;
    
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
        void reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg);

        int16_t torque_i_;
        int16_t vx_i_;
        int16_t vy_i_;
        int16_t r_i_;
        int16_t acc_scaled_;
        int16_t yaw_scaled_;
        int16_t vx_scaled_;
        int16_t vy_scaled_;
        int16_t r_scaled_;
        float state[3];
        float SR[4];
        float TC_out[4];
        float TV_out[4];
        float fx_request;


};