#include "arussim/extended_interface.hpp"

Buttons::Buttons(QWidget* parent) : QWidget(parent), Node("Buttons_Node")
{
    // Set window size
    QScreen* screen = QGuiApplication::primaryScreen();
    setFixedWidth(400);
    setFixedHeight(screen->availableGeometry().height());

    // Set font
    QFont customFont("Montserrat Regular", 13);

    // Reset button
    reset_button_ = new QPushButton("Reset", this);
    reset_button_->move(margins, 500);
    reset_button_->setFixedSize(300, 40);
    reset_button_->setFont(customFont);
    connect(reset_button_, &QPushButton::clicked, this, &Buttons::resetButtonClicked);

    // Telemetry bar
    telemetry_container_ = new QWidget(this);
    telemetry_container_->setFixedSize(50, containerHeight);
    telemetry_container_->setStyleSheet("background-color: lightgray;");
    telemetry_container_->move(margins, margins);

    telemetry_bar_ = new QWidget(telemetry_container_);
    telemetry_bar_->setFixedWidth(50);
    telemetry_bar_->move(0, centerY);
    

    // Publisher
    reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);

    // Subscriber
    cmd_sub_ = this->create_subscription<arussim_msgs::msg::Cmd>(
        "/arussim/cmd", 1, 
        [this](const arussim_msgs::msg::Cmd::SharedPtr msg) { 
            QMetaObject::invokeMethod(this, [this, msg]() {
                updateTelemetryBar(msg->acc);
            }, Qt::QueuedConnection);
        }
    );

    QTimer::singleShot(1000, [this]() {
        raise();
        activateWindow();
    });

}

void Buttons::updateTelemetryBar(double parameter)
{
    double height = std::abs(parameter) * scaleFactor;
    height = std::min(height, maxBarHeight);

    telemetry_bar_->setFixedHeight(static_cast<int>(height));

    if (parameter >= 0) {
        telemetry_bar_->move(telemetry_bar_->x(), centerY - height);
        telemetry_bar_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_->move(telemetry_bar_->x(), centerY);
        telemetry_bar_->setStyleSheet("background-color: red;");
    }
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
