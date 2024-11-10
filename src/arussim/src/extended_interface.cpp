#include "arussim/extended_interface.hpp"
#include <QVBoxLayout>
#include <QApplication>

Buttons::Buttons(QWidget* parent) : QWidget(parent), Node("Buttons_Node")
{
    QVBoxLayout* lReset = new QVBoxLayout(this);
    reset_button_ = new QPushButton("Reset", this);
    connect(reset_button_, &QPushButton::clicked, this, &Buttons::resetButtonClicked);
    lReset->addWidget(reset_button_);
    setLayout(lReset);
    reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);
}

void Buttons::resetButtonClicked()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    reset_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

int main(int argc, char * argv[])
{
    // Initialize both QApplication and rclcpp
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Buttons>();
    node->show();

    std::thread rclcpp_thread([&]() {
        rclcpp::spin(node);
        rclcpp::shutdown();
    });

    int result = app.exec();
    rclcpp_thread.join();

    return result;
}
