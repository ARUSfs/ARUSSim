/**
 * @file supervisor.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com) and Santiago Miranda (santimirandacaballero@gmail.com)
 * @version 0.2
 * @date 2026-02-06
 * 
 * 
 */
#include "arussim/supervisor.hpp"

/**
 * @class Supervisor
 * @brief Constructor for the Supervisor class
 */
Supervisor::Supervisor() : Node("Supervisor")
{
    this->declare_parameter<bool>("csv_supervisor", false);
    this->get_parameter("csv_supervisor", kCSVSupervisor);

    if (kCSVSupervisor) {
        csv_generator_ = std::make_unique<CSVGenerator>("supervisor");
    }

    between_tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arussim/tpl_signal", 10, 
        std::bind(&Supervisor::tpl_signal_callback, this, std::placeholders::_1)
    );

    between_tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arussim/tpl_signal", 10, 
        std::bind(&Supervisor::JSONGenerator_callback, this, std::placeholders::_1)
    );

    hit_cones_sub_ = this->create_subscription<arussim_msgs::msg::PointXY>(
        "/arussim/hit_cones", 10,
        std::bind(&Supervisor::hit_cones_callback, this, std::placeholders::_1)
    );

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Supervisor::reset_callback, this, std::placeholders::_1));

    circuit_sub_ = this->create_subscription<std_msgs::msg::String>("/arussim/circuit", 1, 
        std::bind(&Supervisor:://, this, std::placeholders::_1));

    lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/lap_time", 1);

    hit_cones_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/hit_cones_bool", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Supervisor::timer_callback, this)
    );
}

/**
 * @brief Timer callback to update the timer.
 * 
 */
void Supervisor::timer_callback(){
    speed_multiplier_list_.push_back(simulation_speed_multiplier);
    double sum_ = std::accumulate(speed_multiplier_list_.begin(), speed_multiplier_list_.end(), 0.0);
    mean_ = sum_ / speed_multiplier_list_.size();
}


/**
 * @brief Callback for receiving reset commands.
 * 
 * This method resets the lap times.
 * 
 * @param msg 
 */
void Supervisor::reset_callback([[maybe_unused]]const std_msgs::msg::Bool::SharedPtr msg)
{
    time_list_.clear();
    list_total_hit_cones_.clear();
    hit_cones_lap_.clear();
    speed_multiplier_list_.clear();
    started_ = false;
}

/**
 * @brief Callback to check if the car is between two TPLs.
 * 
 * @param msg 
 */
void Supervisor::tpl_signal_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!started_){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLap started%s", green.c_str(), reset.c_str());
        started_ = true;
    }
    else{
        time_list_.push_back((this->get_clock()->now().seconds() - prev_time_) * mean_);

        std_msgs::msg::Float32 lap_time_msg;
        lap_time_msg.data = time_list_.back();
        lap_time_pub_->publish(lap_time_msg);

        // Detect hit cones in this lap and add to total
        list_total_hit_cones_.push_back(hit_cones_lap_);
        hit_cones_lap_.clear();

        size_t n_total_cones_hit_ = 0;
        for (const auto& i : list_total_hit_cones_) {
            n_total_cones_hit_ += i.size();
        }
        if(n_total_cones_hit_ == 0){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLAP %zu: %f. TOTAL HIT CONES: %zu%s", 
            green.c_str(), time_list_.size(), time_list_.back(), n_total_cones_hit_, reset.c_str());
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sLAP %zu: %f. %sTOTAL HIT CONES: %zu%s", 
            green.c_str(), time_list_.size(), time_list_.back(), yellow.c_str(), n_total_cones_hit_, reset.c_str());
        }
        if (kCSVSupervisor){
            std::vector<std::string> row_values;
            row_values.push_back(std::to_string(lap_time_msg.data));
            row_values.push_back(std::to_string(n_total_cones_hit_));
            csv_generator_->write_row("lap_time,total_hit_cones", row_values);
        }
    }
    prev_time_ = this->get_clock()->now().seconds();
    speed_multiplier_list_.clear();
}

/**
 * @brief Callback to check if the car has hit a cone.
 * 
 * @param msg 
 */
void Supervisor::hit_cones_callback(const arussim_msgs::msg::PointXY::SharedPtr msg)
{
    auto cone_position = std::make_pair(static_cast<double>(msg->x), static_cast<double>(msg->y));

    if (std::find(hit_cones_lap_.begin(), hit_cones_lap_.end(), cone_position) == hit_cones_lap_.end())
    {
        hit_cones_lap_.push_back(cone_position);
        RCLCPP_INFO(this->get_logger(), "%sHit cones: %zu%s", 
        yellow.c_str(), hit_cones_lap_.size(), reset.c_str());

        std_msgs::msg::Bool hit_cones_msg;
        hit_cones_msg.data = true;
        hit_cones_pub_->publish(hit_cones_msg);
    }
}

/**
 * @brief Function to select the best lap time of the run.
 * 
 */
double Supervisor::best_lap()
{
    double lap_time_ = time_list_.push_back((this->get_clock()->now().seconds() - prev_time_) * mean_);
    double real_lap_time_ = lap_time_ + 2 * hit_cones_lap_.size();
    if(prev_time_ + 2*prev_hit_cones_ < real_lap_time_) best_time_ = real_lap_time_;
    size_t prev_hit_cones_ = hit_cones_lap_.size();
    return best_time_;
}
/**
 * @brief Function to create a JSON file of the best times of each track.
 * 
 */
void Supervisor::json_generator_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    lap_time_with_cones = Supervisor::best_lap()
    namespace fs = std::filesystem;
    std::string home_dir = std::string(std::getenv("HOME"));
    std::filesystem::path json_dir = std::filesystem::path(home_dir) / "ws" / "src" / "ARUSSim" / "src" / "arussim" / "laptimes";
    fs::path json_file = json_dir / (circuit_ + ".json");
    if (fs::exists(json_file) && fs::is_regular_file(json_file)) {
        // File already exists
        // -> read it, update it, append to it, etc.
    } 
    else 
    {
        std::ofstream json_out(json_file);
        if (!json_out.is_open()) 
        {
            throw std::runtime_error("Failed to create JSON file");
        }
        json_out << "{\n";
        json_out << "  \"lap_time\": " << time_list_.push_back((this->get_clock()->now().seconds() - prev_time_) * mean_) << ",\n";
        json_out << "  \"cones_hitted\": " << hit_cones_lap_.size() << ",\n";
        json_out << "  \"lap_time_with_cones_hitted\": " << lap_time_with_cones << ",\n";
        json_out << "  \"control_config\": \"" /*<< control_config_ << */"\"\n";
        json_out << "}\n";

        json_out.close();
    }
}
/**
* @brief Callback to store the track name that is selected.
* 
* @param msg 
*/
void track_name(const std_msgs::msg::String::SharedPtr msg)
{
    circuit_ = msg->data;
    const std::string ext = ".pcd";
    if (circuit_.size() >= ext.size() &&
        circuit_.compare(
            circuit_.size() - ext.size(),
            ext.size(),
            ext) == 0)
    {
        circuit_.erase(circuit_.size() - ext.size());
    }
}

/**
 * @brief Main function for the supervisor node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Supervisor>());
    rclcpp::shutdown();
    return 0;
}
