#include <QWidget>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

class Buttons : public QWidget
{
    Q_OBJECT

public:
    explicit Buttons(QWidget* parent = nullptr);

private:
    QPushButton* hello_button;
    void onHelloButtonClicked();
};