/**
 * @file arussim_node.cpp
 * @brief Simulator node for the ARUS Team (ARUSsim)
 */

#include "arussim/arussim_node.hpp"

/**
 * @class Simulator
 * @brief Simulator class for the ARUS Team (ARUSsim).
 * 
 * This class simulates the behavior of a vehicle in a racing environment, 
 * handling the simulation of state updates, sensor data generation, and broadcasting
 * transforms to visualize the vehicle and track in RViz.
 */
Simulator::Simulator() : Node("simulator")
{   
    this->declare_parameter<std::string>("track", "FSG");
    this->declare_parameter<double>("state_update_rate", 1000);
    this->declare_parameter<double>("controller_rate", 100);
    this->declare_parameter<bool>("use_gss", false);
    this->declare_parameter<double>("vehicle.COG_front_dist", 1.9);
    this->declare_parameter<double>("vehicle.COG_back_dist", -1.0);
    this->declare_parameter<double>("vehicle.car_width", 0.8);
    this->declare_parameter<double>("sensor.lidar_fov", 20);
    this->declare_parameter<double>("sensor.camera_fov", 10);
    this->declare_parameter<double>("sensor.pub_rate", 10);
    this->declare_parameter<double>("sensor.noise_position_lidar_perception", 0.01);
    this->declare_parameter<double>("sensor.noise_prob_lidar_perception", 0.2);
    this->declare_parameter<double>("sensor.noise_lidar_color", 0.01);
    this->declare_parameter<double>("sensor.noise_camera_perception", 0.01);
    this->declare_parameter<double>("sensor.noise_camera_color", 0.01);
    this->declare_parameter<double>("sensor.cut_cones_below_x", -1);
    this->declare_parameter<double>("sensor.position_lidar_x", 1.5);
    this->declare_parameter<double>("sensor.position_camera_x", 0.0);
    this->declare_parameter<double>("sensor.camera_color_probability", 0.85);
    this->declare_parameter<bool>("csv_state", false);
    this->declare_parameter<bool>("csv_vehicle_dynamics", false);
    this->declare_parameter<bool>("debug", false);

    this->get_parameter("track", kTrackName);
    this->get_parameter("state_update_rate", kStateUpdateRate);
    this->get_parameter("controller_rate", kControllerRate);
    this->get_parameter("use_gss", kUseGSS);
    this->get_parameter("vehicle.COG_front_dist", kCOGFrontDist);
    this->get_parameter("vehicle.COG_back_dist", kCOGBackDist);
    this->get_parameter("vehicle.car_width", kCarWidth);
    this->get_parameter("sensor.lidar_fov", kLidarFOV);
    this->get_parameter("sensor.camera_fov", kCameraFOV);
    this->get_parameter("sensor.pub_rate", kSensorRate);
    this->get_parameter("sensor.noise_position_lidar_perception", kNoiseLidarPosPerception);
    this->get_parameter("sensor.noise_prob_lidar_perception", kNoiseLidarProbPerception);
    this->get_parameter("sensor.noise_lidar_color", kNoiseLidarColor);
    this->get_parameter("sensor.noise_camera_perception", kNoiseCameraPerception);
    this->get_parameter("sensor.noise_camera_color", kNoiseCameraColor);
    this->get_parameter("sensor.cut_cones_below_x", kMinPerceptionX);
    this->get_parameter("sensor.position_lidar_x", kPosLidarX);
    this->get_parameter("sensor.position_camera_x", kPosCameraX);
    this->get_parameter("sensor.camera_color_probability", kCameraColorProbability);
    this->get_parameter("csv_state", kCSVState);
    this->get_parameter("csv_vehicle_dynamics", kCSVVehicleDynamics);
    this->get_parameter("debug", kDebug);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<arussim_msgs::msg::State>(
        "/arussim/state", 10);
    control_vx_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_vx", 10);
    control_vy_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_vy", 10);
    control_r_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_r", 10);
    track_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/track", 10);
    lidar_perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
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
    camera_perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/camera_perception", 10);

    slow_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kSensorRate)), 
        std::bind(&Simulator::on_slow_timer, this));
    fast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kStateUpdateRate)), 
        std::bind(&Simulator::on_fast_timer, this));
    controller_sim_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kControllerRate)),
        std::bind(&Simulator::on_controller_sim_timer, this));

    cmd_sub_ = this->create_subscription<arussim_msgs::msg::Cmd>("/arussim/cmd", 1, 
        std::bind(&Simulator::cmd_callback, this, std::placeholders::_1));
    circuit_sub_ = this->create_subscription<std_msgs::msg::String>("/arussim/circuit", 1, 
        std::bind(&Simulator::load_track, this, std::placeholders::_1));
    rviz_telep_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&Simulator::rviz_telep_callback, this, std::placeholders::_1));
    
    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Simulator::reset_callback, this, std::placeholders::_1));

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

    // Set controller_sim period and GSS usage
    controller_sim_.init(1/kControllerRate, kUseGSS);

    // Initialize torque variable 
    torque_cmd_ = {0.0, 0.0, 0.0, 0.0};

    // Set CSV file
    if (kCSVState) {
        csv_generator_state_ = std::make_shared<CSVGenerator>("state");
    }
    if (kCSVVehicleDynamics) {
        auto csv_gen = std::make_shared<CSVGenerator>("vehicle_dynamics");
        vehicle_dynamics_.set_csv_generator(csv_gen);
    }

    prev_circuit_ = kTrackName;

    // Call load_track with kTrackName
    auto track_msg = std::make_shared<std_msgs::msg::String>();
    track_msg->data = kTrackName + ".pcd";
    load_track(track_msg);
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
 * @brief Checks if car has started acc event
 * 
 */
void Simulator::check_acc_start(){
    double vx = vehicle_dynamics_.vx_;

    if (vx > 0  && !started_acc_){
        std_msgs::msg::Bool msg;
        msg.data = true;
        lap_signal_pub_->publish(msg);
        started_acc_ = true;

    }
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

    // Random noise generation for position (lidar)
    std::random_device rd_p; 
    std::mt19937 gen_p(rd_p());
    std::normal_distribution<> dist_p(0.0, kNoiseLidarPosPerception);

    // Random noise generation for visualization (lidar)
    std::random_device rd_v; 
    std::mt19937 gen_v(rd_v());
    std::uniform_real_distribution<double> dist_v(0.0, kNoiseLidarProbPerception);  

    // Random noise generation for color (lidar)
    std::random_device rd_c; 
    std::mt19937 gen_c(rd_c());
    std::uniform_real_distribution<double> dist_c(0.0, kNoiseLidarColor);

    // Lidar perception simulation
    auto perception_cloud = pcl::PointCloud<PointXYZProbColorScore>();
    for (auto &point : track_.points)
    {
        double d = std::sqrt(std::pow(point.x - (x + kPosLidarX*std::cos(yaw)), 2) + std::pow(point.y - (y + kPosLidarX*std::sin(yaw)), 2));
        if (d < kLidarFOV)
        {
            PointXYZProbColorScore p;
            p.x = (point.x - x)*std::cos(yaw) + (point.y - y)*std::sin(yaw) + dist_p(gen_p);
            p.y = -(point.x - x)*std::sin(yaw) + (point.y - y)*std::cos(yaw) + dist_p(gen_p);
            p.z = 0.0;
            double p_r = std::exp(-0.005 * d);  // Exponential decay for color
            double a = std::log(2.0) / kLidarFOV; // Calculate 'a' paramater for exponential decay based on max distance
            double p_v = std::exp(-a * d) - dist_v(gen_v); // Exponential decay for visualization
            if (point.color == 0) {
                p.prob_yellow = (point.color + dist_c(gen_c)) * p_r;
                p.prob_blue = (1 - point.color - dist_c(gen_c)) * p_r;
            } else if (point.color == 1) {
                p.prob_yellow = (point.color - dist_c(gen_c)) * p_r;
                p.prob_blue = (1 - point.color + dist_c(gen_c)) * p_r;
            } else {
                p.prob_yellow = 0.0 + dist_c(gen_c);
                p.prob_blue = 0.0 + dist_c(gen_c);
            }
            p.prob_yellow = std::clamp(p.prob_yellow, 0.0, 1.0);
            p.prob_blue   = std::clamp(p.prob_blue, 0.0, 1.0);
            p.score = 1.0;
            if (p.x > kMinPerceptionX && p_v > 0.5) {
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
    lidar_perception_pub_->publish(perception_msg);


    // Random noise generation for position (camera)
    std::random_device rd_pc; 
    std::mt19937 gen_pc(rd_pc());
    std::normal_distribution<> dist_pc(0.0, kNoiseCameraPerception);

    // Random noise generation for color (camera)
    std::random_device rd_cc; 
    std::mt19937 gen_cc(rd_cc());
    std::uniform_real_distribution<double> dist_cc(0.0, kNoiseCameraColor);

    // Camera perception simulation
    auto camera_perception_cloud = pcl::PointCloud<PointXYZProbColorScore>();
    for (auto &point : track_.points)
    {
        double d = std::sqrt(std::pow(point.x - (x + kPosCameraX*std::cos(yaw)), 2) + std::pow(point.y - (y + kPosCameraX*std::sin(yaw)), 2));
        if (d < kCameraFOV)
        {
            PointXYZProbColorScore p;
            p.x = (point.x - x)*std::cos(yaw) + (point.y - y)*std::sin(yaw) + dist_pc(gen_pc);
            p.y = -(point.x - x)*std::sin(yaw) + (point.y - y)*std::cos(yaw) + dist_pc(gen_pc);
            p.z = 0.0;
            if (point.color == 0) {
                p.prob_yellow = (1 - kCameraColorProbability + dist_cc(gen_cc));
                p.prob_blue = (kCameraColorProbability - dist_cc(gen_cc));
            } else if (point.color == 1) {
                p.prob_yellow = (kCameraColorProbability - dist_cc(gen_cc));
                p.prob_blue = (1 - kCameraColorProbability + dist_cc(gen_cc));
            } else {
                p.prob_yellow = 0.0 + dist_cc(gen_cc);
                p.prob_blue = 0.0 + dist_cc(gen_cc);
            }
            p.prob_yellow = std::clamp(p.prob_yellow, 0.0, 1.0);
            p.prob_blue   = std::clamp(p.prob_blue, 0.0, 1.0);
            p.score = 1.0;
            if (p.x > kPosCameraX) {
                camera_perception_cloud.push_back(p);
            }

        }
    }

    sensor_msgs::msg::PointCloud2 camera_perception_msg;
    pcl::toROSMsg(camera_perception_cloud, camera_perception_msg);
    camera_perception_msg.header.stamp = clock_->now();
    camera_perception_msg.header.frame_id = "arussim/vehicle_cog";
    camera_perception_pub_->publish(camera_perception_msg);

    cone_visualization();
}

void Simulator::on_controller_sim_timer() {
    // Update sensor data in ControllerSim
    // TO DO: use sensors with noise instead of ground truth
    controller_sim_.set_sensors(
        vehicle_dynamics_.ax_,
        vehicle_dynamics_.ay_,
        vehicle_dynamics_.r_,
        vehicle_dynamics_.delta_,
        vehicle_dynamics_.wheel_speed_.fl_,
        vehicle_dynamics_.wheel_speed_.fr_,
        vehicle_dynamics_.wheel_speed_.rl_,
        vehicle_dynamics_.wheel_speed_.rr_,
        vehicle_dynamics_.vx_,
        vehicle_dynamics_.vy_
    );

    // Torque command calculation
    torque_cmd_ = controller_sim_.get_torque_cmd(input_acc_, target_r_);
    
    // Publish control estimation 
    std_msgs::msg::Float32 control_vx_msg;
    control_vx_msg.data = controller_sim_.vx_;
    control_vx_pub_->publish(control_vx_msg);

    std_msgs::msg::Float32 control_vy_msg;
    control_vy_msg.data = controller_sim_.vy_;
    control_vy_pub_->publish(control_vy_msg);   

    std_msgs::msg::Float32 control_r_msg;   
    control_r_msg.data = controller_sim_.r_;
    control_r_pub_->publish(control_r_msg);
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

    double dt = 1.0 / kStateUpdateRate;

    vehicle_dynamics_.update_simulation(input_delta_, torque_cmd_, dt);

    if(use_tpl_){
        check_lap();
    }

    if ((kTrackName == "acceleration" && !started_acc_) || (track_name_ == "acceleration" && !started_acc_)){
        check_acc_start();
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

    auto wheel_speeds = arussim_msgs::msg::FourWheelDrive();
    wheel_speeds.front_left = vehicle_dynamics_.wheel_speed_.fl_;
    wheel_speeds.front_right = vehicle_dynamics_.wheel_speed_.fr_;
    wheel_speeds.rear_left = vehicle_dynamics_.wheel_speed_.rl_;
    wheel_speeds.rear_right = vehicle_dynamics_.wheel_speed_.rr_;
    message.wheel_speeds = wheel_speeds;

    auto torque = arussim_msgs::msg::FourWheelDrive();
    torque.front_left = vehicle_dynamics_.torque_cmd_.fl_;
    torque.front_right = vehicle_dynamics_.torque_cmd_.fr_;
    torque.rear_left = vehicle_dynamics_.torque_cmd_.rl_;
    torque.rear_right = vehicle_dynamics_.torque_cmd_.rr_;
    message.torque = torque;

    if (kCSVState){
        std::vector<std::string> row_values;
        row_values.push_back(std::to_string(vehicle_dynamics_.x_));
        row_values.push_back(std::to_string(vehicle_dynamics_.y_));
        row_values.push_back(std::to_string(vehicle_dynamics_.yaw_));
        row_values.push_back(std::to_string(vehicle_dynamics_.vx_));
        row_values.push_back(std::to_string(vehicle_dynamics_.vy_));
        row_values.push_back(std::to_string(vehicle_dynamics_.r_));
        row_values.push_back(std::to_string(vehicle_dynamics_.ax_));
        row_values.push_back(std::to_string(vehicle_dynamics_.ay_));
        row_values.push_back(std::to_string(vehicle_dynamics_.delta_));
        csv_generator_state_->write_row("x,y,yaw,vx,vy,r,ax,ay,delta", row_values);
    }
    if (kCSVVehicleDynamics){
        vehicle_dynamics_.write_csv_row();
    }
    
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
    target_r_ = msg->target_r;
    time_last_cmd_ = clock_->now();
}

void Simulator::reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    input_acc_ = 0.0;
    input_delta_ = 0.0;
    torque_cmd_ = {0.0, 0.0, 0.0, 0.0};

    vehicle_dynamics_ = VehicleDynamics();

    started_acc_ = false;
    if (prev_circuit_ != track_name_){
        fixed_trajectory_msg_.points.clear();
        fixed_trajectory_msg_.s.clear();
        fixed_trajectory_msg_.k.clear();
        fixed_trajectory_msg_.speed_profile.clear();
        fixed_trajectory_msg_.acc_profile.clear();
        auto track_msg = std::make_shared<std_msgs::msg::String>();
        track_msg->data = track_name_ + ".pcd";
        load_track(track_msg);
    }

    // Clear cone markers
    visualization_msgs::msg::MarkerArray empty_markers;
    cone_marker_pub_->publish(empty_markers);

    for (auto &marker : current_cone_markers_.markers) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    cone_marker_pub_->publish(current_cone_markers_);
    current_cone_markers_.markers.clear();
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
void Simulator::load_track(const std_msgs::msg::String::SharedPtr track_msg)
{   
    // Extract and remove the ".pcd" suffix if present
    track_name_ = track_msg->data;
    if(track_name_.size() >= 4 && track_name_.substr(track_name_.size()-4) == ".pcd"){
        track_name_.resize(track_name_.size()-4);
    }

    // Load the track point cloud using the .pcd extension
    std::string package_path = ament_index_cpp::get_package_share_directory("arussim");
    std::string filename = package_path + "/resources/tracks/" + track_name_ + ".pcd";
    if (pcl::io::loadPCDFile<ConeXYZColorScore>(filename, track_) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't read file %s", filename.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded %ld data points from %s", 
                track_.points.size(), filename.c_str());

    // Clear previous TPL cones
    tpl_cones_.clear();

    // Extract the TPLs
    for (const auto& point : track_.points)
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

        // Coordinates of the two cones
        double x1 = tpl_cones_[0].first;
        double y1 = tpl_cones_[0].second;
        double x2 = tpl_cones_[1].first;
        double y2 = tpl_cones_[1].second;

        // Calculate slope (a) and y-intercept (b)
        tpl_coef_a_ = (y2 - y1) / (x2 - x1 + 0.000001);  // Avoid division by zero
        tpl_coef_b_ = y1 - tpl_coef_a_ * x1;

        // Calculate midpoint between the two cones
        mid_tpl_x_ = (x1 + x2) / 2.0;
        mid_tpl_y_ = (y1 + y2) / 2.0;
    }

    // Extract the fixed trajectory using the .json extension
    std::string json_filename = package_path + "/resources/tracks/" + track_name_ + ".json";
    std::ifstream tray_json(json_filename);
    if (!tray_json.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't open JSON trajectory file %s", json_filename.c_str());
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

    if (traj_x.size() == traj_y.size() && traj_x.size() > 0) {
        for (size_t i = 0; i < traj_x.size(); i++) {
            arussim_msgs::msg::PointXY point;
            point.x = traj_x[i];
            point.y = traj_y[i];
            fixed_trajectory_msg_.points.push_back(point);
        }
    }

    if (traj_s.size() == traj_k.size() && traj_s.size() > 0) {
        for (size_t i = 0; i < traj_s.size(); i++) {
            fixed_trajectory_msg_.s.push_back(traj_s[i]);
            fixed_trajectory_msg_.k.push_back(traj_k[i]);
        }
    }

    if (traj_speed_profile.size() > 0) {
        for (size_t i = 0; i < traj_speed_profile.size(); i++) {
            fixed_trajectory_msg_.speed_profile.push_back(traj_speed_profile[i]);
        }
    }

    if (traj_acc_profile.size() > 0) {
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

    // Store them to delete later
    current_cone_markers_ = cone_markers;
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
