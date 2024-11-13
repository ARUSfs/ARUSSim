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
    this->declare_parameter<double>("vehicle.COG_front_dist", 1.9);
    this->declare_parameter<double>("vehicle.COG_back_dist", -1.0);
    this->declare_parameter<double>("vehicle.car_width", 0.8);
    this->declare_parameter<double>("sensor.fov_radius", 20);
    this->declare_parameter<double>("sensor.position_lidar_x", 1.8);
    this->declare_parameter<double>("sensor.pub_rate", 10);
    this->declare_parameter<double>("sensor.noise_sigma", 0.01);
    this->declare_parameter<double>("sensor.cut_cones_below_x", -1);

    this->get_parameter("track", kTrackName);
    this->get_parameter("friction_coef", kFrictionCoef);
    this->get_parameter("state_update_rate", kStateUpdateRate);
    this->get_parameter("vehicle.COG_front_dist", kCOGFrontDist);
    this->get_parameter("vehicle.COG_back_dist", kCOGBackDist);
    this->get_parameter("vehicle.car_width", kCarWidth);
    this->get_parameter("sensor.fov_radius", kFOV);
    this->get_parameter("sensor.position_lidar_x", kPosLidarX);
    this->get_parameter("sensor.pub_rate", kSensorRate);
    this->get_parameter("sensor.noise_sigma", kNoisePerception);
    this->get_parameter("sensor.cut_cones_below_x", kMinPerceptionX);


    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<arussim_msgs::msg::State>(
        "/arussim/state", 10);
    track_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/track", 10);
    perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/perception", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/arussim/vehicle_visualization", 1);
    cone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/arussim/cone_visualization", 1);
    lap_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/arussim/tpl_signal", 10);
    hit_cones_pub_ = this->create_publisher<arussim_msgs::msg::PointXY>(
        "/arussim/hit_cones", 10);
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
    
    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Simulator::reset_callback, this, std::placeholders::_1));

    set_fov_service_ = this->create_service<arussim_msgs::srv::SetFOV>(
        "arussim/set_fov",
        std::bind(&Simulator::handle_set_fov, this, 
            std::placeholders::_1, std::placeholders::_2));

    // Load the car mesh
    marker_.header.frame_id = "arussim/vehicle_cog";
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://arussim/resources/meshes/ART24D.stl";
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
 * @brief Service handler for setting the FOV.
 * 
 * This method updates the FOV parameter based on a service request.
 * 
 * @param request The service request message.
 * @param response The service response message.
 */
void Simulator::handle_set_fov(
    const std::shared_ptr<arussim_msgs::srv::SetFOV::Request> request,
    std::shared_ptr<arussim_msgs::srv::SetFOV::Response> response)
{
    try {
        kFOV = request->fov;
        response->success = true;
        response->message = "FOV updated successfully to " + std::to_string(kFOV);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error updating FOV: " + std::string(e.what());
        RCLCPP_ERROR(get_logger(), "Error updating FOV: %s", e.what());
    }
}


/**
 * @brief Determines if the vehicle is above, below, or on the line formed by the two cones.
 * 
 * @param tpl_cones_ Vector of two points (cones), where each point is represented as a pair (x, y).
 */
void Simulator::check_lap() {
    double x = vehicle_dynamics_.x_;
    double y = vehicle_dynamics_.y_;

    // Calculate the signed distance from the vehicle to the line equation through the TPLs
    double dist_to_tpl = y - tpl_coef_a_ * x - tpl_coef_b_;

    // Calculate the distance from the vehicle to the midpoint between the TPLs
    double d = std::sqrt(std::pow(x - mid_tpl_x_, 2) + std::pow(y - mid_tpl_y_, 2));

    if (dist_to_tpl * prev_dist_to_tpl_ < 0 and d < 5) {
        // Publish the result
        std_msgs::msg::Bool msg;
        msg.data = true;
        lap_signal_pub_->publish(msg);
    } 
    prev_dist_to_tpl_ = dist_to_tpl;
}

/**
 * @brief Slow timer callback for sensor data updates.
 * 
 * This method updates the sensor data by publishing the track and generating 
 * random noise to simulate sensor inaccuracy. It also publishes the perception data.
 */
void Simulator::on_slow_timer()
{   
    double x = vehicle_dynamics_.x_;
    double y = vehicle_dynamics_.y_;
    double yaw = vehicle_dynamics_.yaw_;

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
        double d = std::sqrt(std::pow(point.x - (x + kPosLidarX*std::cos(yaw)), 2) + std::pow(point.y - (y + kPosLidarX*std::sin(yaw)), 2));
        if (d < kFOV)
        {
            ConeXYZColorScore p;
            p.x = (point.x - x)*std::cos(yaw) + (point.y - y)*std::sin(yaw) + dist(gen);
            p.y = -(point.x - x)*std::sin(yaw) + (point.y - y)*std::cos(yaw) + dist(gen);
            p.z = 0.0;
            p.color = point.color;
            p.score = 1.0;
            if (p.x > kMinPerceptionX) {
                perception_cloud.push_back(p);
            }
            if (p.x >= kCOGBackDist && p.x <= kCOGFrontDist && p.y >= -kCarWidth && p.y <= kCarWidth)
            {
                arussim_msgs::msg::PointXY msg;
                msg.x = point.x;
                msg.y = point.y;
                hit_cones_pub_->publish(msg);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 perception_msg;
    pcl::toROSMsg(perception_cloud, perception_msg);
    perception_msg.header.stamp = clock_->now();
    perception_msg.header.frame_id="arussim/vehicle_cog";
    perception_pub_->publish(perception_msg);

    cone_visualization();
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
    rclcpp::Time current_time = clock_->now();
    if((current_time - time_last_cmd_).seconds() > 0.2 && vehicle_dynamics_.vx_ != 0)
    {
        input_acc_ = 0;
    }

    double dt = 1/kStateUpdateRate;

    vehicle_dynamics_.update_simulation(input_delta_, input_acc_, dt);

    if(use_tpl_){
        check_lap();
    }
    
    auto message = arussim_msgs::msg::State();
    message.x = vehicle_dynamics_.x_;
    message.y = vehicle_dynamics_.y_;
    message.yaw = vehicle_dynamics_.yaw_;
    message.vx = vehicle_dynamics_.vx_;
    message.vy = vehicle_dynamics_.vy_;
    message.r = vehicle_dynamics_.r_;
    message.ax = vehicle_dynamics_.ax_;
    message.ay = vehicle_dynamics_.ay_;
    message.delta = vehicle_dynamics_.delta_;
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

void Simulator::reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    vehicle_dynamics_ = VehicleDynamics();
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

    vehicle_dynamics_.x_ = msg->pose.pose.position.x;
    vehicle_dynamics_.y_ = msg->pose.pose.position.y;
    vehicle_dynamics_.yaw_ = yaw;
    vehicle_dynamics_.vx_ = 0;
    vehicle_dynamics_.vy_ = 0;
    vehicle_dynamics_.r_ = 0;
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
    transform_stamped.transform.translation.x = vehicle_dynamics_.x_;
    transform_stamped.transform.translation.y = vehicle_dynamics_.y_;
    transform_stamped.transform.translation.z = 0.0;

    // Set the rotation (using a quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, vehicle_dynamics_.yaw_); // Roll, Pitch, Yaw in radians
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
        double x1 = tpl_cones_[0].first;
        double y1 = tpl_cones_[0].second;
        double x2 = tpl_cones_[1].first;
        double y2 = tpl_cones_[1].second;

        // Calculate the slope (a) and y-intercept (b)
        tpl_coef_a_ = (y2 - y1) / (x2 - x1 + 0.000001);  // Avoid division by zero in case of vertically aligned cones
        tpl_coef_b_ = y1 - tpl_coef_a_ * x1;

        // Calculate the midpoint between the two cones
        mid_tpl_x_ = (x1 + x2) / 2.0;
        mid_tpl_y_ = (y1 + y2) / 2.0;
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
        for (size_t i = 0; i < traj_x.size(); i++) {
            arussim_msgs::msg::PointXY point;
            point.x = traj_x[i];
            point.y = traj_y[i];
            fixed_trajectory_msg_.points.push_back(point);
        }
    }

    if(traj_s.size() == traj_k.size() && traj_s.size() > 0) {
        for (size_t i = 0; i < traj_s.size(); i++) {
            fixed_trajectory_msg_.s.push_back(traj_s[i]);
            fixed_trajectory_msg_.k.push_back(traj_k[i]);
        }
    }

    if(traj_speed_profile.size() > 0) {
        for (size_t i = 0; i < traj_speed_profile.size(); i++) {
            fixed_trajectory_msg_.speed_profile.push_back(traj_speed_profile[i]);
        }
    }

    if(traj_acc_profile.size() > 0) {
        for (size_t i = 0; i < traj_acc_profile.size(); i++) {
            fixed_trajectory_msg_.acc_profile.push_back(traj_acc_profile[i]);
        }
    }

}


/**
 * @brief Make a MarkerArray of all cones of the track
 * 
 */
void Simulator::cone_visualization(){
    visualization_msgs::msg::MarkerArray cone_markers;
    int id_counter = 0;

    for (const auto& point : track_.points) {
        visualization_msgs::msg::Marker cone_marker;
        cone_marker.header.frame_id = "arussim/world";
        cone_marker.ns = "arussim/cones";
        cone_marker.id = id_counter++;  // unique ID for each cone
        cone_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        cone_marker.mesh_resource = "package://arussim/resources/meshes/cone.stl";
        cone_marker.action = visualization_msgs::msg::Marker::ADD;
        cone_marker.pose.position.x = point.x;
        cone_marker.pose.position.y = point.y;
        cone_marker.pose.position.z = 0.0;
        cone_marker.scale.x = 0.001;
        cone_marker.scale.y = 0.001;
        cone_marker.scale.z = 0.001;
        if (point.color == 0) {
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.0;
            cone_marker.color.b = 1.0;
        } else if (point.color == 1){
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 1.0;
            cone_marker.color.b = 0.0;
        } else if (point.color == 2){
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.65;
            cone_marker.color.b = 0.0;
        } else if (point.color == 3){
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.65;
            cone_marker.color.b = 0.0;
        } else if (point.color == 4){
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.5;
            cone_marker.color.b = 0.0;
        }
        cone_marker.color.a = 1.0;
        cone_marker.lifetime = rclcpp::Duration::from_seconds(0.0); // Infinite lifetime

        cone_markers.markers.push_back(cone_marker);
    }

    cone_marker_pub_->publish(cone_markers);
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
