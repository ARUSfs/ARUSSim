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

    hit_cones_sub_ = this->create_subscription<arussim_msgs::msg::PointXY>(
        "/arussim/hit_cones", 10,
        std::bind(&Supervisor::hit_cones_callback, this, std::placeholders::_1)
    );

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Supervisor::reset_callback, this, std::placeholders::_1));

    circuit_sub_ = this->create_subscription<std_msgs::msg::String>("/arussim/circuit", 1, 
        std::bind(&Supervisor::track_name, this, std::placeholders::_1));

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
    std::filesystem::path ws_path = std::filesystem::canonical("/proc/self/exe");
    for (int i = 0; i < 5; ++i) ws_path = ws_path.parent_path();
    std::filesystem::path dir_path = ws_path / "src/ARUSSim/src/arussim/laptimes";
    std::filesystem::path file_path = dir_path / (circuit_ + ".csv");

    // Reads the file if it exists and overwrites it if the time done in simulation is better than the stored one.
    if (std::filesystem::exists(file_path) && std::filesystem::is_regular_file(file_path))
    {
        if(!started_){
            std::ifstream file(file_path);

            if (!file.is_open()) {
                throw std::runtime_error("Could not open CSV file");
            }
            std::string line;
            std::string last_line;

            std::getline(file, line);

            while (std::getline(file, line)) {
                if (!line.empty()){
                    last_line = line;
                    break;
                }
            }

                file.close();

                if (!last_line.empty()) {
                std::stringstream ss(last_line);
                std::string lap, cones, time;
                std::getline(ss, lap, ',');
                std::getline(ss, cones, ',');
                std::getline(ss, time, ',');

                prev_best_time_ = std::stod(time);
                }
            }
        else{
            if(prev_best_time_ > best_time_){
                std::ofstream file(file_path);
                file << "lap,cones,best_time\n";
                file << lap_time_ << "," << cones_hitted_ << "," << best_time_ << "," << "\n" << "\"" << controller_dump_ << "\"" << "\n";
                file.close();
            }
        }
    }

    else // Creates a .csv file and adds the time done in simulation.
    {
        if(started_){
            std::ofstream file(file_path, std::ios::out);
            if (!file.is_open()) {
                throw std::runtime_error("Could not create CSV file");
            }

            file << "lap,cones,best_time\n";
            file << lap_time_ << "," << cones_hitted_ << "," << best_time_ << "," << "\n" << "\"" << controller_dump_ << "\"" << "\n";
        }
    }

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

        double real_lap_time_ = time_list_.back() + 2 * hit_cones_lap_.size();
        if(prev_time_ + 2 * prev_hit_cones_ > real_lap_time_)
        {
        best_time_ = real_lap_time_;
        cones_hitted_ = hit_cones_lap_.size();
        lap_time_ = time_list_.back();
        }

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
    prev_hit_cones_ = list_total_hit_cones_.size() - prev_hit_cones_;
    prev_time_ = this->get_clock()->now().seconds();
    speed_multiplier_list_.clear();
    controller_dump_ = exec_command();
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
 * @brief Callback to store the track name that is selected.
 * 
 * @param msg 
 */
void Supervisor::track_name(const std_msgs::msg::String::SharedPtr msg)
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
 * @brief Function to return a string with config params.
 * 
 * @return std::string
 */
std::string Supervisor::exec_command()
{
    std::array<char, 256> buffer;
    std::string result;
    // Execute the command on bash to get the controller parameters and stores it in a string.
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return "Error executing the command.";

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
        result += buffer.data();

    pclose(pipe);
    return result;
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
