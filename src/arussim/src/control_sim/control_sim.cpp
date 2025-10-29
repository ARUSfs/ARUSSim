/**
 * @file controller_sin.cpp
 * @author Carmen Menenendez (carmenmenendezda@gmail.com)
 * @brief contol_sim node for the ARUSSIM package. This node simulates the Control PCB of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/control_sim/control_sim.hpp"

/**
 * @class ControlSim
 * @brief ControlSim class for ARUSsim
 * This class simulates the Control PCB of the vehicle.
 * 
 */

ControlSim::ControlSim() : Node("control_sim") {

    this->declare_parameter<double>("mass", 348.0);
    this->declare_parameter<double>("rdyn", 0.225);
    this->declare_parameter<double>("gear_ratio", 12.48);
    this->get_parameter("mass", kMass);
    this->get_parameter("rdyn", kRdyn);
    this->get_parameter("gear_ratio", kGearRatio);

    // Timers
    timer_receive_ = this->create_wall_timer(std::chrono::milliseconds(1),
        std::bind(&ControlSim::receive_can, this));
    timer_send_ = this->create_wall_timer(std::chrono::milliseconds(5),
        std::bind(&ControlSim::send_torque, this));
    timer_state_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&ControlSim::send_state, this));

    // CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_.ifr_name, "can0");
    ioctl(can_socket_, SIOCGIFINDEX, &ifr_);
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_));

        
}

 void ControlSim::receive_can()
{
    
   read(can_socket_, &frame, sizeof(struct can_frame));

    if (frame.can_id == 0x222) { 
        acc_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
        yaw_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);

        acc_ = acc_scaled_ / 100.0;
        target_r_ = yaw_scaled_ / 1000.0;
        float max_torque = 21 * 1000 / 9.8;

        float F = acc_ * kMass;
        torque_i_ = (int)(F * 0.225 / 4.0 * 1000.0 / 9.8);
        RCLCPP_INFO(this->get_logger(), "F = %.2f, torque_i = %d", F, torque_i_);
    }

    else if (frame.can_id == 0x1A3) {
        vx_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
        vy_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);

        vx_ = vx_scaled_ * 0.2f;
        vy_ = vy_scaled_ * 0.2f;

    }
    else if (frame.can_id == 0x1A4) {
        r_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);

        r_ = r_scaled_ * 0.00034906585;

    }
    
    
}

void ControlSim::send_torque()
{
   std::array<uint32_t, 4> ids = {0x200, 0x203, 0x206, 0x209};

    for (auto id : ids) {
        frame.can_id = id;
        frame.can_dlc = 8;
        frame.data[0] = 0x00;
        frame.data[1] = 0x07;

        frame.data[2] = torque_i_ & 0xFF;
        frame.data[3] = (torque_i_ >> 8) & 0xFF;

        uint16_t torque_max = 1000;
        uint16_t torque_min = -1000;
        frame.data[4] = torque_max & 0xFF;
        frame.data[5] = (torque_max >> 8) & 0xFF;
        frame.data[6] = torque_min & 0xFF;
        frame.data[7] = (torque_min >> 8) & 0xFF;

        write(can_socket_, &frame, sizeof(struct can_frame));


        }
       
}

void ControlSim::send_state(){
    
    frame.can_id = 0x122;
    frame.can_dlc = 6;

    vx_i_ = (int)(vx_ * 100.0f);
    frame.data[0] = vx_i_ & 0xFF;
    frame.data[1] = (vx_i_ >> 8) & 0xFF;

    vy_i_ = (int)(vy_ * 100.0f);
    frame.data[2] = vy_i_ & 0xFF;
    frame.data[3] = (vy_i_ >> 8) & 0xFF;

    r_i_ = (int)(r_ * 1000.0f);
    frame.data[4] = r_i_ & 0xFF;
    frame.data[5] = (r_i_ >> 8) & 0xFF;

    write(can_socket_, &frame, sizeof(struct can_frame));

    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlSim>());
    rclcpp::shutdown();
    return 0;
}