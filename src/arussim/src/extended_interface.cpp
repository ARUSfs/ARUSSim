#include "arussim/extended_interface.hpp"
#include <QVBoxLayout>
#include <QApplication>

Buttons::Buttons(QWidget* parent) : QWidget(parent)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    hello_button = new QPushButton("Hello World", this);
    connect(hello_button, &QPushButton::clicked, this, &Buttons::onHelloButtonClicked);
    layout->addWidget(hello_button);
    setLayout(layout);
}

void Buttons::onHelloButtonClicked()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello World");
}

int main(int argc, char * argv[])
{
    // Initialize both QApplication and rclcpp
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Buttons>();
    node->show();

    // Spin rclcpp in a separate thread
    std::thread rclcpp_thread([]() {
        rclcpp::spin(rclcpp::Node::make_shared("arussim_buttons"));
        rclcpp::shutdown();
    });

    int result = app.exec();
    rclcpp_thread.join(); // Wait for rclcpp thread to finish

    return result;
}
