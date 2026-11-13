/**
 * @file ideal_slam.cpp
 * @brief Ideal SLAM for ARUSSim: replaces graph_slam using the simulator ground truth.
 *
 * Publishes the same interface as graph_slam:
 *   - TF arussim/world -> slam/vehicle  (perfect pose)
 *   - /slam/map                         (cones detected by the simulated lidar, with perfect position)
 *   - /slam/lap_count                   (laps completed)
 */
#include <cmath>
#include <vector>

#include <pcl/point_types.h>
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


// Cone colors, same values as in the rest of the stack
constexpr int kBlue = 0;
constexpr int kYellow = 1;
constexpr int kUncolored = -1;

class IdealSlam : public rclcpp::Node
{
public:
    IdealSlam() : Node("ideal_slam")
    {
        this->declare_parameter<std::string>("map_topic", "/slam/map");
        this->declare_parameter<std::string>("lap_count_topic", "/slam/lap_count");
        this->declare_parameter<std::string>("track_topic", "/arussim/track");
        this->declare_parameter<std::string>("perception_topic", "/arussim/perception");
        this->declare_parameter<std::string>("ground_truth_topic", "/arussim/ground_truth");
        this->declare_parameter<std::string>("tpl_signal_topic", "/arussim/tpl_signal");
        this->declare_parameter<std::string>("reset_topic", "/arussim/reset");
        this->declare_parameter<std::string>("global_frame", "arussim/world");
        this->declare_parameter<std::string>("local_frame", "slam/vehicle");
        this->declare_parameter<double>("sensor_max_dist", 30.0);
        this->declare_parameter<double>("match_dist", 1.0);
        this->declare_parameter<double>("track_tolerance", 0.001);
        this->declare_parameter<double>("publish_freq", 10.0);
        this->declare_parameter<int>("min_observations", 12);
        this->declare_parameter<double>("min_lap_distance", 30.0);
        global_frame_ = this->get_parameter("global_frame").as_string();
        local_frame_ = this->get_parameter("local_frame").as_string();
        max_dist_ = this->get_parameter("sensor_max_dist").as_double();
        match_dist_ = this->get_parameter("match_dist").as_double();
        track_tolerance_ = this->get_parameter("track_tolerance").as_double();
        min_observations_ = this->get_parameter("min_observations").as_int();
        min_lap_distance_ = this->get_parameter("min_lap_distance").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("map_topic").as_string(), 10);
        lap_count_pub_ = this->create_publisher<std_msgs::msg::Int16>(this->get_parameter("lap_count_topic").as_string(), 10);

        track_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("track_topic").as_string(), 1, std::bind(&IdealSlam::track_callback, this, std::placeholders::_1));
        perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("perception_topic").as_string(), 1, std::bind(&IdealSlam::perception_callback, this, std::placeholders::_1));
        ground_truth_sub_ = this->create_subscription<common_msgs::msg::State>(
            this->get_parameter("ground_truth_topic").as_string(), 1, std::bind(&IdealSlam::ground_truth_callback, this, std::placeholders::_1));
        tpl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            this->get_parameter("tpl_signal_topic").as_string(), 10, std::bind(&IdealSlam::tpl_callback, this, std::placeholders::_1));
        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            this->get_parameter("reset_topic").as_string(), 1, [this](std_msgs::msg::Bool::SharedPtr) { reset(); });

        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / this->get_parameter("publish_freq").as_double()), std::bind(&IdealSlam::on_timer, this));
    }

private:
    void reset()
    {
        observations_.assign(track_.size(), 0);
        in_map_.assign(track_.size(), false);
        map_.clear();
        passed_.clear();
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
            changed = std::abs(new_track[i].x - track_[i].x) > track_tolerance_
                   || std::abs(new_track[i].y - track_[i].y) > track_tolerance_;
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
        tf.header.frame_id = global_frame_;
        tf.child_frame_id = local_frame_;
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
        // Same criteria as graph_slam: a new lap needs a minimum driven distance
        if (driven_distance_ > min_lap_distance_) {
            driven_distance_ = 0.0;
            lap_count_++;
        }
    }

    void perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_pose_) return;
        pcl::PointCloud<pcl::PointXYZ> scan;
        pcl::fromROSMsg(*msg, scan);
        const double c = std::cos(yaw_), s = std::sin(yaw_);

        std::vector<bool> counted(track_.size(), false);
        for (const auto& p : scan) {
            if (std::hypot(p.x, p.y) > max_dist_) continue;  // Farther than the sensor range we want to simulate
            const double wx = x_ + p.x * c - p.y * s;
            const double wy = y_ + p.x * s + p.y * c;

            size_t best = track_.size();
            double best_dist = match_dist_;
            for (size_t i = 0; i < track_.size(); ++i) {
                const double d = std::hypot(track_[i].x - wx, track_[i].y - wy);
                if (d < best_dist) { best_dist = d; best = i; }
            }
            if (best == track_.size() || counted[best]) continue;
            counted[best] = true;
            observations_[best]++;

            // Like graph_slam, a cone is published once it has been seen several times
            if (in_map_[best] || observations_[best] < min_observations_) continue;
            in_map_[best] = true;

            SlamCone cone{};
            cone.x = track_[best].x;
            cone.y = track_[best].y;
            cone.z = 0;
            cone.color = kUncolored;  // Uncolored until the car passes it (see on_timer)
            cone.score = track_[best].score;
            map_.push_back(cone);
            passed_.push_back(false);
        }
    }

    void on_timer()
    {
        // Same as graph_slam (pass_coloring): a cone is colored when the car has just passed it
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        for (size_t i = 0; i < map_.size(); ++i) {
            if (passed_[i]) continue;
            const double dx = map_[i].x - x_, dy = map_[i].y - y_;
            const double lx = dx * c + dy * s, ly = -dx * s + dy * c;
            if (std::hypot(lx, ly) < 6.0 && lx < 0.0 && lx > -2.0) {
                map_[i].color = ly > 0.0 ? kBlue : kYellow;
                passed_[i] = true;
            }
        }

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(map_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = global_frame_;
        map_pub_->publish(map_msg);

        std_msgs::msg::Int16 lap_msg;
        lap_msg.data = lap_count_;
        lap_count_pub_->publish(lap_msg);
    }

    std::string global_frame_, local_frame_;
    int min_observations_;
    double max_dist_, match_dist_, track_tolerance_, min_lap_distance_;
    pcl::PointCloud<ConeXYZColorScore> track_;
    pcl::PointCloud<SlamCone> map_;
    std::vector<bool> passed_;  // Cone already colored because the car passed it
    std::vector<int> observations_;
    std::vector<bool> in_map_;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, driven_distance_ = 0.0;
    bool has_pose_ = false;
    int16_t lap_count_ = 0;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr lap_count_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr track_sub_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr ground_truth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
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
