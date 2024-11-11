#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QApplication>
#include <QScreen>
#include <QFont>
#include <QTimer>


#include <rclcpp/rclcpp.hpp>

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

    double scaleFactor = 100;
    double maxBarHeight = 200;
    double containerHeight = 300;
    double centerY = containerHeight / 2;
    double margins = 50;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;

    rclcpp::Subscription<arussim_msgs::msg::Cmd>::SharedPtr cmd_sub_;


    void resetButtonClicked();
    void updateTelemetryBar(double parameter);


    //Loginfo colors
    const std::string red = "\033[1;31m";
    const std::string green = "\033[1;32m";
    const std::string yellow = "\033[1;33m";
    const std::string blue = "\033[1;34m";
    const std::string magenta = "\033[1;35m";
    const std::string cyan = "\033[1;36m";
    const std::string reset = "\033[0m";

};