/**
 * @file sensors.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Sensors node for the ARUSSIM package. This node simulates the sensors of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/sensors.hpp"
std::vector<Sensors::CanFrame> Sensors::frames;
/**
 * @class Sensors
 * @brief Sensors class for ARUSsim
 * This class simulates some sensors of the vehicle like the IMU, wheel speed sensors and extensometer.
 */
Sensors::Sensors() : Node("sensors")
{
    // Declare and get noise parameters for each GSS variable
    this->declare_parameter<double>("gss.noise_gss_vx", 0.01);
    this->declare_parameter<double>("gss.noise_gss_vy", 0.01);
    this->declare_parameter<double>("gss.noise_imu_ax", 0.01);
    this->declare_parameter<double>("gss.noise_imu_ay", 0.01);
    this->declare_parameter<double>("gss.noise_imu_r", 0.025);
    this->declare_parameter<double>("gss.gss_frequency", 100.0);

    // Declare inverter parameters
    this->declare_parameter<double>("inverter.noise_motor_speed_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_rear_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_left", 0.01);
    this->declare_parameter<double>("inverter.gear_ratio", 12.48);
    this->declare_parameter<double>("inverter.inverter_frequency", 100.0);

    // Declare extensometer parameters
    this->declare_parameter<double>("extensometer.extensometer_frequency", 100.0);
    this->declare_parameter<double>("extensometer.noise_extensometer", 0.01);


    // Get parameters
    this->get_parameter("gss.noise_gss_vx", kNoiseGssVx);
    this->get_parameter("gss.noise_imu_ax", kNoiseGssVy);
    this->get_parameter("gss.noise_imu_ax", kNoiseImuAx);
    this->get_parameter("gss.noise_imu_ay", kNoiseImuAy);
    this->get_parameter("gss.noise_imu_r", kNoiseImuR);
    this->get_parameter("gss.gss_frequency", kGssFrequency);

    this->get_parameter("inverter.noise_motor_speed_front_right", kNoiseMotorSpeedFrontRight);
    this->get_parameter("inverter.noise_motor_speed_front_left", kNoiseMotorSpeedFrontLeft);
    this->get_parameter("inverter.noise_motor_speed_rear_right", kNoiseMotorSpeedRearRight);
    this->get_parameter("inverter.noise_motor_speed_rear_left", kNoiseMotorSpeedRearLeft);
    this->get_parameter("inverter.noise_torque_front_right", kNoiseTorqueFrontRight);
    this->get_parameter("inverter.noise_torque_front_left", kNoiseTorqueFrontLeft);
    this->get_parameter("inverter.noise_torque_rear_right", kNoiseTorqueRearRight);
    this->get_parameter("inverter.noise_torque_rear_left", kNoiseTorqueRearLeft);
    this->get_parameter("inverter.gear_ratio", kGearRatio);
    this->get_parameter("inverter.inverter_frequency", kInverterFrequency);

    this->get_parameter("extensometer.extensometer_frequency", kExtensometerFrequency);
    this->get_parameter("extensometer.noise_extensometer", kNoiseExtensometer);

    //CAN Socket setup
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_.ifr_name, "can0");
    ioctl(can_socket_, SIOCGIFINDEX, &ifr_);
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_));

    // State subscriber
    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // IMU
    gss_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kGssFrequency)),
        std::bind(&Sensors::gss_timer, this)
    );


    ext_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kExtensometerFrequency)),
        std::bind(&Sensors::extensometer_timer, this)
    );

    // Inverter
    torque_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>("/arussim/torque4WD", 10);

    inverter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kInverterFrequency)),
        std::bind(&Sensors::inverter_timer, this)
    );

    frames = {
        {0x1A0, 4, {
            {"GSS/vx", {0, 15, true, 0.00555556, 0.0}},
            {"GSS/vy", {16, 31, true, 0.00555556, 0.0}}
        }},
        {0x1A3, 4, { // GSS
            {"IMU/ax", {0, 15, true, 0.02, 0.0}},
            {"IMU/ay", {16, 31, true, 0.02, 0.0}},
            
        }},
        {0x1A4, 6, { // IMU yaw_rate
            {"IMU/yaw_rate", {32, 47, true, 0.000349065, 0.0}}
        }},
        {0x134, 2, { // Extensometer
            {"extensometer", {0, 15, true, -0.000031688042484, 0.476959989071}}
        }},
        {0x102, 5, { // Front Left inverter
            {"fl_inv_speed", {0, 23, true, 0.0001047197551196, 0.0}},
            {"fl_inv_torque", {24, 39, true, 0.0098, 0.0}}
        }},
        {0x106, 5, { // Front Right inverter
            {"fr_inv_speed", {0, 23, true, 0.0001047197551196, 0.0}},
            {"fr_inv_torque", {24, 39,  true, 0.0098, 0.0}}
        }},
        {0x110, 5, { // Rear Left inverter
            {"rl_inv_speed", {0, 23, true, 0.0001047197551196, 0.0}},
            {"rl_inv_torque", {24, 39,  true, 0.0098, 0.0}}
        }},
        {0x114, 5, { // Rear Right inverter
            {"rr_inv_speed", {0, 23, true, 0.0001047197551196, 0.0}},
            {"rr_inv_torque", {24, 39, true, 0.0098, 0.0}}
        }},
        {0x161, 2, {
            {"as_status", {8, 15, false, 1.0, 0.0}}
        }}
    };
}

/**
 * @brief Callback function for the state subscriber
 * 
 * @param msg 
 */
void Sensors::state_callback(const arussim_msgs::msg::State::SharedPtr msg)
{
    // Update state variables with incoming data
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
    vx_ = msg->vx;
    vy_ = msg->vy;
    r_ = msg->r;
    ax_ = msg->ax;
    ay_ = msg->ay;
    delta_ = msg->delta;
    wheel_speed_msg = msg->wheel_speeds;
    torque_cmd_msg = msg->torque;
}

//Function to encode a signal into a CAN frame
uint64_t Sensors::encode_signal(double value, double scale, double offset, int bit_len, bool /*is_signed*/)
{
    double raw_f_ = (value - offset) / scale;
    int64_t raw_i_ = (int64_t) std::round(raw_f_);
    uint64_t mask_ = (1ULL << (bit_len)) - 1;
    raw_i_ &= mask_;
    return (uint64_t)raw_i_;  
}

/**
 * @brief Timer function for the 4WD torque
 * 
 */
void Sensors::inverter_timer(){
    // Random noise generation with different noise for each wheel speed
    std::random_device rd; 
    std::mt19937 gen(rd());

    // Corrected noise distributions
    std::normal_distribution<> dist_fr(0.0, kNoiseTorqueFrontRight);
    std::normal_distribution<> dist_fl(0.0, kNoiseTorqueFrontLeft);
    std::normal_distribution<> dist_rr(0.0, kNoiseTorqueRearRight);
    std::normal_distribution<> dist_rl(0.0, kNoiseTorqueRearLeft);

    // Apply noise to the state variables
    torque_cmd_.fr_ = torque_cmd_msg.front_right;       // 2WD
    torque_cmd_.fl_ = torque_cmd_msg.front_left;        // 2WD
    torque_cmd_.rr_ = torque_cmd_msg.rear_right + dist_rr(gen);
    torque_cmd_.rl_ = torque_cmd_msg.rear_left + dist_rl(gen);

    // Create the torque message
    auto message = arussim_msgs::msg::FourWheelDrive();

    message.front_right = torque_cmd_.fr_;    
    message.front_left = torque_cmd_.fl_;      
    message.rear_right = torque_cmd_.rr_;     
    message.rear_left = torque_cmd_.rl_;     

    // Publish the torque message
    torque_pub_->publish(message);

     // Random noise generation with different noise for each wheel speed
    std::normal_distribution<> dist_front_right(0.0, kNoiseMotorSpeedFrontLeft);
    std::normal_distribution<> dist_front_left(0.0, kNoiseMotorSpeedFrontRight);
    std::normal_distribution<> dist_rear_right(0.0, kNoiseMotorSpeedRearLeft);
    std::normal_distribution<> dist_rear_left(0.0, kNoiseMotorSpeedRearRight);

    // Apply noise to the state variables
    wheel_speed_.fr_ = wheel_speed_msg.front_right;         // 2WD
    wheel_speed_.fl_ = wheel_speed_msg.front_left;          // 2WD
    wheel_speed_.rr_ = wheel_speed_msg.rear_right + std::abs(dist_rear_right(gen));
    wheel_speed_.rl_ = wheel_speed_msg.rear_left + std::abs(dist_rear_left(gen));

    //Send Inverter CAN frames
     std::map<std::string,double> values = { {"fl_inv_speed", wheel_speed_.fl_*kGearRatio}, {"fl_inv_torque", torque_cmd_.fl_/kGearRatio}, 
    {"fr_inv_speed", wheel_speed_.fr_*kGearRatio}, {"fr_inv_torque", torque_cmd_.fr_/kGearRatio}, 
    {"rl_inv_speed", wheel_speed_.rl_*kGearRatio}, {"rl_inv_torque", torque_cmd_.rl_/kGearRatio}, 
    {"rr_inv_speed", wheel_speed_.rr_*kGearRatio}, {"rr_inv_torque", torque_cmd_.rr_/kGearRatio} 
    }; 

    for (auto &frame : frames) { if (frame.id == 0x102 || frame.id == 0x106 || frame.id == 0x110 || frame.id == 0x114) { 
        send_can_frame(frame, values); 
    } 
} 
}

/**
 * @brief Timer function for the GSS
 * 
 */
void Sensors::gss_timer()
{
    // Random noise generation with different noise for each variable
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> dist_vx(0.0, kNoiseGssVx);
    std::normal_distribution<> dist_vy(0.0, kNoiseGssVy);
    std::normal_distribution<> dist_ax(0.0, kNoiseImuAx);
    std::normal_distribution<> dist_ay(0.0, kNoiseImuAy);
    std::normal_distribution<> dist_r(0.0, kNoiseImuR);

    std::map<std::string,double> values = { {"IMU/ax", ax_ + dist_ax(gen)}, 
    {"IMU/ay", ay_ + dist_ay(gen)}, {"IMU/yaw_rate", r_ + dist_r(gen)}, 
    {"GSS/vx", vx_ + dist_vx(gen)}, {"GSS/vy", vy_ + dist_vy(gen)} }; 

    for (auto &frame : frames) { 
        if (frame.id == 0x1A3 || frame.id == 0x1A4 || frame.id == 0x1A0)  { 
        send_can_frame(frame, values); 
    } 
}

}


/**
 * @brief Timer function for the extensometer
 * 
 */
void Sensors::extensometer_timer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseExtensometer);

    //Send CAN frame for extensometer
    std::map<std::string,double> values = { {"extensometer", delta_ + dist(gen)} }; 

    for (auto &frame : frames) { 
        if (frame.id == 0x134) { 
            send_can_frame(frame, values); 
        }
    } 
}

void Sensors::send_can_frame(const CanFrame &frame, const std::map<std::string,double> &values) { 
    canMsg_.can_id = frame.id; 
    canMsg_.can_dlc = frame.size; 
    std::memset(canMsg_.data, 0, sizeof(canMsg_.data)); 
    for (auto &sig_pair : frame.signals) { 
        const auto &topic = sig_pair.first; 
        const auto &sig = sig_pair.second; 
        int bit_len = sig.bit_fin - sig.bit_in + 1; 
        uint64_t raw = encode_signal(values.at(topic), sig.scale, sig.offset, bit_len, sig.is_signed); 
        int bits_written = 0;

        while (bits_written < bit_len) {
            int byte_idx = (sig.bit_in + bits_written) / 8;
            int bit_offset = (sig.bit_in + bits_written) % 8;
            int bits_in_this_byte = std::min(8 - bit_offset, bit_len - bits_written);
            canMsg_.data[byte_idx] |= ((raw >> bits_written) & ((1 << bits_in_this_byte) - 1)) << bit_offset;
            bits_written += bits_in_this_byte;
}

    } 
    write(can_socket_, &canMsg_, sizeof(canMsg_)); 
}

/**
 * @brief Main function
 * 
 * This initializes the ROS 2 system and starts spinning the Sensor node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application. 
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}