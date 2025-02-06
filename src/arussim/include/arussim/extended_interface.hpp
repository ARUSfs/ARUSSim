#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>
#include <QProcess>

namespace rviz_panel_tutorial
{
class ExtendedInterface : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ExtendedInterface(QWidget * parent = 0);
  ~ExtendedInterface() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void topicCallback(const std_msgs::msg::String & msg);

  QLabel* label_;
  QPushButton* button_;

  QProcess* simulation_process_ = nullptr;

private Q_SLOTS:
  void buttonActivated();
};
}  // namespace rviz_panel_tutorial