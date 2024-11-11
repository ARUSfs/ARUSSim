#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QApplication>
#include <QScreen>
#include <QFont>
#include <QTimer>
#include <QSlider>
#include <QLabel>


#include <rclcpp/rclcpp.hpp>
#include "arussim_msgs/srv/set_fov.hpp"
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

    QSlider* fov_setter_;
    QLabel* fov_label_;

    QSlider* a_setter_;
    QLabel* a_label_;

    QSlider* b_setter_;
    QLabel* b_label_;

    double scaleFactor_ = 100;
    double maxBarHeight_ = 150;
    double containerHeight_ = 150;
    double containerWidth_ = 150;
    double centerY_ = containerHeight_ / 2;
    double margins_ = 50;

    double kFOV = 20;
    double kA;
    double kB;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;

    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_;

    rclcpp::Client<arussim_msgs::srv::SetFOV>::SharedPtr fov_client_;


    void reset_button_clicked();
    void a_button_clicked();
    void b_button_clicked();

    void update_telemetry_bar(double fl_param_, double fr_param_, double rl_param_, double rr_param_);
    void update_telemetry_labels(double vx_, double vy_, double r_, double ax_, double ay_, double delta_);

    void fov_set(int value_);
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