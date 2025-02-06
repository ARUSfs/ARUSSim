#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/display_context.hpp>

#include <QLabel>
#include <QPushButton>
#include <QProcess>
#include <QVBoxLayout>
#include <QElapsedTimer>
#include <QVector>
#include <QPair>
#include <QPixmap>
#include <QComboBox>  // nuevo include

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "arussim_msgs/msg/state.hpp"


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

  QLabel* best_lt_label_;
  QLabel* last_lt_label_;
  QLabel* hit_cones_label_;
  QLabel* lap_label_;

  QPushButton* launch_button_;
  QPushButton* stop_button_;
  QPushButton* reset_button_;

  QLabel* telemetry_label_;
  QWidget* telemetry_container_fl_;
  QWidget* telemetry_bar_fl_;
  QWidget* telemetry_container_fr_;
  QWidget* telemetry_bar_fr_;
  QWidget* telemetry_container_rl_;
  QWidget* telemetry_bar_rl_;
  QWidget* telemetry_container_rr_;
  QWidget* telemetry_bar_rr_;

  QProcess* simulation_process_ = nullptr;

  QElapsedTimer timer_;
  QVector<QPair<double, double>> vx_history_;
  QLabel* vx_graph_label_ = nullptr;

  QComboBox* selection_box_ = nullptr;

private Q_SLOTS:
  void launch_button_clicked();
  void stop_button_clicked();
  void reset_button_clicked();
  void update_lap_time_labels(double lap_time_);
  void update_telemetry_bar(double fr_param_, double fl_param_, double rr_param_, double rl_param_);
  void update_telemetry_labels(double vx_, double vy_, double r_, double ax_, double ay_, double delta_);
  void circuit_selector(const QString & option);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr circuit_pub_;

  rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr torque_sub_;
  rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hit_cones_bool_sub_;

  double best_lap_time_ = 9999.99;
  double last_lap_time_ = 9999.99;
  int hit_cones_counter_ = 0;
  int lap_counter_ = 0;

  double max_bar_height_ = 300;
  double max_torque_value_ = 90.0;
  double scale_factor_ = max_bar_height_ / max_torque_value_;
  double center_y_ = max_bar_height_ / 2;

  double min_vx_ = 0.0;
  double max_vx_ = 25.0;


  //Loginfo colors
  const std::string red = "\033[1;31m";
  const std::string green = "\033[1;32m";
  const std::string yellow = "\033[1;33m";
  const std::string blue = "\033[1;34m";
  const std::string magenta = "\033[1;35m";
  const std::string cyan = "\033[1;36m";
  const std::string reset = "\033[0m";
};
}  // namespace rviz_panel_tutorial