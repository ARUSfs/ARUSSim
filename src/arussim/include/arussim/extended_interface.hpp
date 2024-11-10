#include <QWidget>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"

class Buttons : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    Buttons(QWidget* parent = nullptr);

private:
    QPushButton* reset_button_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
    
    void resetButtonClicked();
};