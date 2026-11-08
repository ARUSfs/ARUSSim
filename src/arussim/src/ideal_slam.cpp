/**
 * @file ideal_slam.cpp
 * @brief Ideal SLAM for ARUSSim: replaces graph_slam using the simulator ground truth.
 *
 * Publishes the same interface as graph_slam:
 *   - TF arussim/world -> slam/vehicle  (perfect pose)
 *   - /slam/map                         (cones already seen by the car, perfect position and color)
 *   - /slam/lap_count                   (laps completed)
 */
#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <common_msgs/msg/state.hpp>

#include "arussim/ConeXYZColorScore.h"

// Cone published in /slam/map. The stack reads the covariances, which are 0 here (ideal SLAM).
struct SlamCone {
    PCL_ADD_POINT4D;
    int color;
    float score;
    float cov_xx;
    float cov_yy;
    float cov_xy;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (SlamCone,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, color, color)
                                   (float, score, score)
                                   (float, cov_xx, cov_xx)
                                   (float, cov_yy, cov_yy)
                                   (float, cov_xy, cov_xy)
)

constexpr double kMinConeDist = 0.1;  // m  Min distance between two cones of the map

class IdealSlam : public rclcpp::Node
{
public:
    IdealSlam() : Node("ideal_slam")
    {
        this->declare_parameter<double>("sensor_max_dist", 40.0);   // m   Max range to see a cone
        this->declare_parameter<double>("sensor_fov", 120.0);       // deg Field of view to see a cone
        this->declare_parameter<double>("min_lap_distance", 30.0);  // m   Min distance to count a new lap
        max_dist_ = this->get_parameter("sensor_max_dist").as_double();
        fov_ = this->get_parameter("sensor_fov").as_double() * M_PI / 180.0;
        min_lap_distance_ = this->get_parameter("min_lap_distance").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map", 10);
        lap_count_pub_ = this->create_publisher<std_msgs::msg::Int16>("/slam/lap_count", 10);

        track_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arussim/track", 1, std::bind(&IdealSlam::track_callback, this, std::placeholders::_1));
        ground_truth_sub_ = this->create_subscription<common_msgs::msg::State>(
            "/arussim/ground_truth", 1, std::bind(&IdealSlam::ground_truth_callback, this, std::placeholders::_1));
        tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arussim/tpl_signal", 10, std::bind(&IdealSlam::tpl_callback, this, std::placeholders::_1));
        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arussim/reset", 1, [this](std_msgs::msg::Bool::SharedPtr) { reset(); });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IdealSlam::on_timer, this));
    }

private:
    void reset()
    {
        seen_.assign(track_.size(), false);
        map_.clear();
        lap_count_ = 0;
        driven_distance_ = 0.0;
    }

    void track_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<ConeXYZColorScore> new_track;
        pcl::fromROSMsg(*msg, new_track);

        // The simulator republishes the track periodically: only reset when it changes
        bool changed = new_track.size() != track_.size();
        for (size_t i = 0; !changed && i < new_track.size(); ++i) {
            changed = std::abs(new_track[i].x - track_[i].x) > 1e-3
                   || std::abs(new_track[i].y - track_[i].y) > 1e-3;
        }
        if (changed) {
            track_ = new_track;
            reset();
        }
    }

    void ground_truth_callback(const common_msgs::msg::State::SharedPtr msg)
    {
        if (has_pose_) driven_distance_ += std::hypot(msg->x - x_, msg->y - y_);
        x_ = msg->x;
        y_ = msg->y;
        yaw_ = msg->yaw;
        has_pose_ = true;

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "arussim/world";
        tf.child_frame_id = "slam/vehicle";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }

    void tpl_callback(const std_msgs::msg::Bool::SharedPtr)
    {
        // Same criterion as graph_slam: a new lap needs a minimum driven distance
        if (driven_distance_ > min_lap_distance_) {
            driven_distance_ = 0.0;
            lap_count_++;
        }
    }

    void on_timer()
    {
        // Add the cones inside the sensor range and field of view
        for (size_t i = 0; i < track_.size(); ++i) {
            const double dx = track_[i].x - x_;
            const double dy = track_[i].y - y_;
            const double angle = std::remainder(std::atan2(dy, dx) - yaw_, 2 * M_PI);
            if (seen_[i] || std::hypot(dx, dy) > max_dist_ || std::abs(angle) > fov_ / 2.0) continue;

            seen_[i] = true;

            // Like graph_slam, do not add a cone if there is already one in the map at the same place
            // (the triangulation in path_planning fails with duplicated points)
            const bool duplicated = std::any_of(map_.begin(), map_.end(), [&](const SlamCone& c) {
                return std::hypot(c.x - track_[i].x, c.y - track_[i].y) < kMinConeDist;
            });
            if (duplicated) continue;

            SlamCone cone{};
            cone.x = track_[i].x;
            cone.y = track_[i].y;
            cone.z = track_[i].z;
            cone.data[3] = 1.0f;
            // Like the simulated perception, only blue and yellow are colored (orange and TPL cones are uncolored)
            cone.color = (track_[i].color == 0 || track_[i].color == 1) ? track_[i].color : -1;
            cone.score = track_[i].score;
            map_.push_back(cone);
        }

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(map_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "arussim/world";
        map_pub_->publish(map_msg);

        std_msgs::msg::Int16 lap_msg;
        lap_msg.data = lap_count_;
        lap_count_pub_->publish(lap_msg);
    }

    double max_dist_, fov_, min_lap_distance_;
    pcl::PointCloud<ConeXYZColorScore> track_;
    pcl::PointCloud<SlamCone> map_;
    std::vector<bool> seen_;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, driven_distance_ = 0.0;
    bool has_pose_ = false;
    int16_t lap_count_ = 0;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr lap_count_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr track_sub_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr ground_truth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tpl_sub_, reset_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IdealSlam>());
    rclcpp::shutdown();
    return 0;
}
