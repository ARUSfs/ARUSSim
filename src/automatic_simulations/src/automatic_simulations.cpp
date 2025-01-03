#include "automatic_simulations/automatic_simulations.hpp"

/**
 * @brief Construct a new Automatic Simulations:: Automatic Simulations object
 * 
 * @param parent 
 */
AutomaticSimulations::AutomaticSimulations() : Node("automatic_simulations")
{
    // Declare parameters
    this->declare_parameter<double>("simulation_speed_multiplier", 1.0);
    this->declare_parameter<std::string>("event", "Trackdrive");
    this->declare_parameter<double>("laps_target", 1.0);

    // Get parameters
    this->get_parameter("simulation_speed_multiplier", kSimulationSpeedMultiplier);
    this->get_parameter("event", kEvent);
    this->get_parameter("laps_target", kLapsTarget);

    // Competition logic
    if (kEvent == "Trackdrive"){
        kLapsTarget = 10;
    } else if (kEvent == "Acceleration"){
        kLapsTarget = 1;
    } else if (kEvent == "Skidpad"){
        kLapsTarget = 4;
    } else if (kEvent == "AutoX"){
        kLapsTarget = 1;
    }

    RCLCPP_INFO(this->get_logger(), "Starting automatic simulations for %s", kEvent.c_str());
    RCLCPP_INFO(this->get_logger(), "Simulation speed multiplier: %f", kSimulationSpeedMultiplier);
    RCLCPP_INFO(this->get_logger(), "Laps target: %f", kLapsTarget);

    // Publishers
    reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);

    // Subscribers
    lap_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/lap_time", 1, 
        std::bind(&AutomaticSimulations::lap_time_callback, this, std::placeholders::_1)
    );
    
}

/**
 * @brief Callback function for the lap time subscriber
 * 
 * @param msg 
 */
void AutomaticSimulations::lap_time_callback([[maybe_unused]]const std_msgs::msg::Float32::SharedPtr msg){
    current_laps_ ++;
    if (current_laps_ >= kLapsTarget){
        RCLCPP_INFO(this->get_logger(), "%sAll laps completed%s", cyan.c_str(), reset.c_str());
        rclcpp::shutdown();
        exit(0);
    }
}

/**
 * @brief Main function
 * 
 * This initializes the ROS 2 system and starts spinning the Sensor node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return int Return code.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutomaticSimulations>());
    rclcpp::shutdown();
    return 0;
}