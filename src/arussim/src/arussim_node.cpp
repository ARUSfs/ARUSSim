/**
 * @file arussim_node.cpp
 * @brief Simulator node for the ARUS Team (ARUSim)
 */

#include "arussim/arussim_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "arussim/sensors.hpp"
#include <random>
#include <nlohmann/json.hpp>

/**
 * @class Simulator
 * @brief Simulator class for the ARUS Team (ARUSim).
 * 
 * This class simulates the behavior of a vehicle in a racing environment, 
 * handling the simulation of state updates, sensor data generation, and broadcasting
 * transforms to visualize the vehicle and track in RViz.
 */
Simulator::Simulator() : Node("simulator")
{   
    this->declare_parameter<std::string>("track", "FSG.pcd");
    this->declare_parameter<double>("friction_coef", 0.5);
    this->declare_parameter<double>("state_update_rate", 100);
    this->declare_parameter<double>("mass", 200);
    this->declare_parameter<double>("wheel_base", 1.5);
    this->declare_parameter<double>("sensor.fov_radius", 20);
    this->declare_parameter<double>("sensor.pub_rate", 10);
    this->declare_parameter<double>("sensor.noise_sigma", 0.01);
    this->declare_parameter<double>("sensor.cut_cones_below_x", -1);

    this->get_parameter("track", kTrackName);
    this->get_parameter("friction_coef", kFrictionCoef);
    this->get_parameter("state_update_rate", kStateUpdateRate);
    this->get_parameter("mass", kMass);
    this->get_parameter("wheel_base", kWheelBase);
    this->get_parameter("sensor.fov_radius", kFOV);
    this->get_parameter("sensor.pub_rate", kSensorRate);
    this->get_parameter("sensor.noise_sigma", kNoisePerception);
    this->get_parameter("sensor.cut_cones_below_x", kMinPerceptionX);


    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<arussim_msgs::msg::State>("/arussim/state", 10);
    track_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arussim/track", 10);
    perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/perception", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/arussim/vehicle_visualization", 1);
    lap_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/tpl_signal", 10);
    fixed_trajectory_pub_ = this->create_publisher<arussim_msgs::msg::Trajectory>(
        "/arussim/fixed_trajectory", 10);

    slow_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kSensorRate)), 
        std::bind(&Simulator::on_slow_timer, this));
    fast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kStateUpdateRate)), 
        std::bind(&Simulator::on_fast_timer, this));

    cmd_sub_ = this->create_subscription<arussim_msgs::msg::Cmd>("/arussim/cmd", 1, 
        std::bind(&Simulator::cmd_callback, this, std::placeholders::_1));
    rviz_telep_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&Simulator::rviz_telep_callback, this, std::placeholders::_1));


    // Load the car mesh
    marker_.header.frame_id = "arussim/vehicle_cog";
    marker_.header.stamp = clock_->now();
    marker_.ns = "arussim";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://arussim/resources/meshes/whole_car.stl";
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = 1.0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.scale.x = 0.001;
    marker_.scale.y = 0.001;
    marker_.scale.z = 0.001;
    marker_.color.r = 60.0/255.0;
    marker_.color.g = 55.0/255.0;
    marker_.color.b = 70.0/255.0;
    marker_.color.a = 1.0;
    marker_.lifetime = rclcpp::Duration::from_seconds(0.0);

    // Load the track pointcloud
    load_track(track_);
}


/**
 * @brief Determines if the vehicle is above, below, or on the line formed by the two cones.
 * 
 * @param tpl_cones_ Vector of two points (cones), where each point is represented as a pair (x, y).
 */
void Simulator::check_lap() {
    // Calculate the current value of the line equation
    current_position = y_ - a * x_ - b;

    // Calculate the distance from the vehicle to the midpoint between the TPLs
    distance_to_midpoint = std::sqrt(std::pow(x_ - mid_x, 2) + std::pow(y_ - mid_y, 2));

    if (current_position * prev_position < 0 and distance_to_midpoint<5) {
        // Publish the result
        std_msgs::msg::Bool msg;
        msg.data = true;
        lap_signal_pub_->publish(msg);
    } 
    prev_position = current_position;
}

/**
 * @brief Slow timer callback for sensor data updates.
 * 
 * This method updates the sensor data by publishing the track and generating 
 * random noise to simulate sensor inaccuracy. It also publishes the perception data.
 */
void Simulator::on_slow_timer()
{   
    // Update track
    sensor_msgs::msg::PointCloud2 track_msg;
    pcl::toROSMsg(track_, track_msg);
    track_msg.header.stamp = clock_->now();
    track_msg.header.frame_id="arussim/world";
    track_pub_->publish(track_msg);

    fixed_trajectory_pub_->publish(fixed_trajectory_msg_);

    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoisePerception);

    auto perception_cloud = pcl::PointCloud<ConeXYZColorScore>();
    for (auto &point : track_.points)
    {
        double d = std::sqrt(std::pow(point.x - x_, 2) + std::pow(point.y - y_, 2));
        if (d < kFOV)
        {
            ConeXYZColorScore p;
            p.x = (point.x - x_)*std::cos(yaw_) + (point.y - y_)*std::sin(yaw_) + dist(gen);
            p.y = -(point.x - x_)*std::sin(yaw_) + (point.y - y_)*std::cos(yaw_) + dist(gen);
            p.z = 0.0;
            p.color = point.color;
            p.score = 1.0;
            if (p.x > kMinPerceptionX) {
                perception_cloud.push_back(p);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 perception_msg;
    pcl::toROSMsg(perception_cloud, perception_msg);
    perception_msg.header.stamp = clock_->now();
    perception_msg.header.frame_id="arussim/vehicle_cog";
    perception_pub_->publish(perception_msg);
}

/**
 * @brief Fast timer callback for state updates.
 * 
 * This method handles vehicle state updates, broadcasting the transform 
 * and updating the vehicle marker in RViz.
 */
void Simulator::on_fast_timer()
{   
    // Update state and broadcast transform
    update_state();

    if(use_tpl_){
        check_lap();
    }
    
    auto message = arussim_msgs::msg::State();
    message.x = x_;
    message.y = y_;
    message.yaw = yaw_;
    message.vx = vx_;
    message.vy = vy_;
    message.r = r_;
    state_pub_->publish(message);

    broadcast_transform();
    
    // Update vehicle marker
    marker_.header.stamp = clock_->now();
    marker_pub_->publish(marker_);
}

/**
 * @brief Callback for receiving control commands.
 * 
 * This method updates the vehicle's acceleration and steering angle based on 
 * incoming commands.
 * 
 * @param msg Incoming control command message.
 */
void Simulator::cmd_callback(const arussim_msgs::msg::Cmd::SharedPtr msg)
{
    input_acc_ = msg->acc;
    input_delta_ = msg->delta;
    time_last_cmd_ = clock_->now();
}

/**
 * @brief Callback for receiving RViz teleportation commands.
 * 
 * This method teleports the vehicle to a new pose based on incoming RViz commands.
 * 
 * @param msg Incoming pose message from RViz.
 */
void Simulator::rviz_telep_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = yaw;
    vx_ = 0;
    vy_ = 0;
    r_ = 0;
}

/**
 * @brief Updates the vehicle state based on the current input and model equations.
 * 
 * This method updates the position, velocity, and orientation of the vehicle.
 */
void Simulator::update_state()
{
    rclcpp::Time current_time = clock_->now();
    if((current_time - time_last_cmd_).seconds() > 0.2 && vx_ != 0)
    {
        input_acc_ = vx_ > 0 ? -kFrictionCoef*9.8 : kFrictionCoef*9.8;
    }

    double dt = 0.01;

    // Model equations
    double x_dot = vx_ * std::cos(yaw_);
    double y_dot = vx_ * std::sin(yaw_);
    double yaw_dot = vx_ / kWheelBase * std::tan(input_delta_);
    
    // Update state
    x_ += x_dot * dt;
    y_ += y_dot * dt;
    yaw_ += yaw_dot * dt;
    vx_ += input_acc_ * dt;
}

/**
 * @brief Broadcasts the transform of the vehicle to the TF tree.
 * 
 * This method sends the current pose of the vehicle to the ROS TF system
 * to be visualized in RViz or used in other nodes.
 */
void Simulator::broadcast_transform()
{
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set the frame ID and child frame ID
    transform_stamped.header.stamp = clock_->now();
    transform_stamped.header.frame_id = "arussim/world";
    transform_stamped.child_frame_id = "arussim/vehicle_cog";

    // Set the translation (x, y, z)
    transform_stamped.transform.translation.x = x_;
    transform_stamped.transform.translation.y = y_;
    transform_stamped.transform.translation.z = 0.0;

    // Set the rotation (using a quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_); // Roll, Pitch, Yaw in radians
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);
}

/**
 * @brief Filters the track point cloud to extract the TPLs.
 * 
 * @param track 
 */
void Simulator::load_track(const pcl::PointCloud<ConeXYZColorScore>& track)
{   
    std::string package_path = ament_index_cpp::get_package_share_directory("arussim");
    std::string filename = package_path+"/resources/tracks/"+kTrackName+".pcd";
    if (pcl::io::loadPCDFile<ConeXYZColorScore>(filename, track_) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't read file %s", filename.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded %ld data points from %s", 
                track_.points.size(), filename.c_str());

    // Extract the TPLs
    for (const auto& point : track.points)
    {
        if (point.color == 4)
        {
            tpl_cones_.push_back(std::make_pair(point.x, point.y));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TPLs: %ld", tpl_cones_.size());

    if (tpl_cones_.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "tpl_cones_ does not contain exactly 2 points.");
        use_tpl_ = false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TPL1: (%f, %f), TPL2: (%f, %f)", 
                    tpl_cones_[0].first, tpl_cones_[0].second, tpl_cones_[1].first, tpl_cones_[1].second);
        use_tpl_ = true;

        // Coordinates of the two cones (tpl_cones)
        x1 = tpl_cones_[0].first;
        y1 = tpl_cones_[0].second;
        x2 = tpl_cones_[1].first;
        y2 = tpl_cones_[1].second;

        // Calculate the slope (a) and y-intercept (b)
        a = (y2 - y1) / (x2 - x1 + 0.000001);  // Avoid division by zero in case of vertically aligned cones
        b = y1 - a * x1;

        // Calculate the midpoint between the two cones
        mid_x = (x1 + x2) / 2.0;
        mid_y = (y1 + y2) / 2.0;
    }


    // Extract the fixed trajectory
    std::string json_filename = package_path+"/resources/tracks/"+kTrackName+".json";
    std::ifstream tray_json(json_filename);
    if (!tray_json.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't open JSON trayectory file %s", json_filename.c_str());
        return;
    }

    nlohmann::json tray_data;
    tray_json >> tray_data;
    tray_json.close();

    std::vector<float> traj_x = tray_data["x"].get<std::vector<float>>();
    std::vector<float> traj_y = tray_data["y"].get<std::vector<float>>();
    std::vector<float> traj_s = tray_data["s"].get<std::vector<float>>();
    std::vector<float> traj_k = tray_data["k"].get<std::vector<float>>();
    std::vector<float> traj_speed_profile = tray_data["speed_profile"].get<std::vector<float>>();
    std::vector<float> traj_acc_profile = tray_data["acc_profile"].get<std::vector<float>>();

    if(traj_x.size() == traj_y.size() && traj_x.size() > 0) {
        for (int i = 0; i < traj_x.size(); i++) {
            arussim_msgs::msg::PointXY point;
            point.x = traj_x[i];
            point.y = traj_y[i];
            fixed_trajectory_msg_.points.push_back(point);
        }
    }

    if(traj_s.size() == traj_k.size() && traj_s.size() > 0) {
        for (int i = 0; i < traj_s.size(); i++) {
            fixed_trajectory_msg_.s.push_back(traj_s[i]);
            fixed_trajectory_msg_.k.push_back(traj_k[i]);
        }
    }

    if(traj_speed_profile.size() > 0) {
        for (int i = 0; i < traj_speed_profile.size(); i++) {
            fixed_trajectory_msg_.speed_profile.push_back(traj_speed_profile[i]);
        }
    }

    if(traj_acc_profile.size() > 0) {
        for (int i = 0; i < traj_acc_profile.size(); i++) {
            fixed_trajectory_msg_.acc_profile.push_back(traj_acc_profile[i]);
        }
    }

}



/**
 * @brief Main entry point for the simulator node.
 * 
 * This initializes the ROS 2 system and starts spinning the Simulator node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
