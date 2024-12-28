#include "automatic_simulations/automatic_simulations.hpp"

/**
 * @brief Construct a new Automatic Simulations:: Automatic Simulations object
 * 
 * @param parent 
 */
AutomaticSimulations::AutomaticSimulations() : Node("automatic_simulations")
{

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