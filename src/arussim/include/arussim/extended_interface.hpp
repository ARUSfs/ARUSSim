#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QApplication>
#include <QScreen>
#include <QFont>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QProcess>

#include <rclcpp/rclcpp.hpp>
#include "arussim_msgs/srv/set_timer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "arussim_msgs/msg/state.hpp"
#include "arussim_msgs/msg/cmd.hpp"
#include "std_msgs/msg/bool.hpp"

class ExtendedInterface : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    ExtendedInterface(QWidget* parent = nullptr);

private:
    QPushButton* reset_button_;
    QPushButton* a_button_;
    QPushButton* b_button_;

    QLabel* last_lt_label_;
    QLabel* best_lt_label_;
    QLabel* hit_cones_label_;
    QLabel* lap_label_;

    QLabel* telemetry_label_;
    QWidget* telemetry_container_fl_;
    QWidget* telemetry_bar_fl_;
    QWidget* telemetry_container_fr_;
    QWidget* telemetry_bar_fr_;
    QWidget* telemetry_container_rl_;
    QWidget* telemetry_bar_rl_;
    QWidget* telemetry_container_rr_;
    QWidget* telemetry_bar_rr_;
    QLabel* vx_label_;
    QLabel* vy_label_;
    QLabel* ax_label_;
    QLabel* ay_label_;
    QLabel* r_label_;
    QLabel* delta_label_;

    QSlider* timer_setter_;
    QLabel* timer_label_;

    QSlider* a_setter_;
    QLabel* a_label_;

    QSlider* b_setter_;
    QLabel* b_label_;

    QProcess* simulation_process_ = nullptr;

    double scale_factor_ = 100;
    double telemetry_container_height_;
    double window_width_ = QGuiApplication::primaryScreen()->availableGeometry().width() * 0.25;
    double window_height_ = QGuiApplication::primaryScreen()->availableGeometry().height();
    double window_position_x_ = QGuiApplication::primaryScreen()->availableGeometry().width() - window_width_;
    double max_bar_height_ = window_height_ * 0.15;
    double center_y_ = max_bar_height_ / 2;
    double margins_ = window_width_ * 0.05;

    double telemetry_rear_container_position_y_;
    double telemetry_parameters_position_y_;
    double reset_button_position_y_;
    double timer_setter_position_y_;
    double ab_button_position_y_;

    double simulation_speed_multiplier = 1.0;
    double kA;
    double kB;

    double best_lap_time_ = 9999.99;
    double last_lap_time_ = 9999.99;
    int hit_cones_counter_ = 0;
    int lap_counter_ = 0;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;

    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hit_cones_bool_sub_;
    rclcpp::Client<arussim_msgs::srv::SetTimer>::SharedPtr timer_client_;


    void reset_button_clicked();
    void launch_button_clicked();
    void stop_button_clicked();

    void update_telemetry_bar(double fl_param_, double fr_param_, double rl_param_, double rr_param_);
    void update_telemetry_labels(double vx_, double vy_, double r_, double ax_, double ay_, double delta_);
    void update_lap_time_labels(double lap_time_);

    void timer_set(int value_);
    void a_set(int value_);
    void b_set(int value_);


    //Loginfo colors
    const std::string red = "\033[1;31m";
    const std::string green = "\033[1;32m";
    const std::string yellow = "\033[1;33m";
    const std::string blue = "\033[1;34m";
    const std::string magenta = "\033[1;35m";
    const std::string cyan = "\033[1;36m";
    const std::string reset = "\033[0m";

};