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

class Buttons : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    Buttons(QWidget* parent = nullptr);

private:
    QPushButton* reset_button_;
    QWidget* telemetry_bar_;
    QWidget* telemetry_container_;
    QSlider* fov_setter_;
    QLabel* fov_label_;
    QLabel* telemetry_label_;
    QLabel* vx_label_;
    QLabel* vy_label_;
    QLabel* ax_label_;
    QLabel* ay_label_;
    QLabel* r_label_;
    QLabel* delta_label_;

    double scaleFactor = 100;
    double maxBarHeight = 200;
    double containerHeight = 300;
    double centerY = containerHeight / 2;
    double margins = 50;

    double kFOV;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;

    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_;

    rclcpp::Client<arussim_msgs::srv::SetFOV>::SharedPtr fov_client_;


    void resetButtonClicked();
    void updateTelemetryBar(double parameter);
    void updateTelemetryLabels(double vx, double vy, double r, double ax, double ay, double delta);
    void fovValueChanged(int value);


    //Loginfo colors
    const std::string red = "\033[1;31m";
    const std::string green = "\033[1;32m";
    const std::string yellow = "\033[1;33m";
    const std::string blue = "\033[1;34m";
    const std::string magenta = "\033[1;35m";
    const std::string cyan = "\033[1;36m";
    const std::string reset = "\033[0m";

};