#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
#include <QComboBox>
#include <QPainter>
#include <QPen>
#include <QPainterPath>
#include <QDir>
#include <QGuiApplication>
#include <QScreen>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QScrollArea>


#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "arussim_msgs/msg/state.hpp"


namespace plot_interface
{
class PlotInterface : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PlotInterface(QWidget * parent = 0);
  ~PlotInterface() override;

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
  QElapsedTimer timer_gg_;
  QVector<QPair<double, double>> vx_history_;
  QVector<QPair<double, double>> target_speed_history_;
  QLabel* speed_graph_label_ = nullptr;

  QVector<std::tuple<double, double, double>> gg_vector_; // (ay, ax, vx)
  QLabel* gg_graph_label_ = nullptr;

  QLabel* vx_label_;
  QLabel* vy_label_;
  QLabel* r_label_;
  QLabel* ax_label_;
  QLabel* ay_label_;  
  QLabel* delta_label_;

  QSize screen_size_;

  QComboBox* circuit_select_ = nullptr;
  QComboBox* launch_select_ = nullptr;

  QPoint gg_last_mouse_pos_;

  QLabel* gg_legend_label_;

private Q_SLOTS:
  void launch_button_clicked();
  void reset_callback();
  void update_telemetry_bar(double fr_param_, double fl_param_, double rr_param_, double rl_param_);
  void update_vx_target_graph();
  void update_gg_graph();
  void update_telemetry_labels();
  void state_callback(double vx_, double vy_, double r_, double ax_, double ay_, double delta_);
  void zoom_in_speed_graph();
  void zoom_out_speed_graph();
  void zoom_in_gg_graph();
  void zoom_out_gg_graph();

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr torque_sub_;
  rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hit_cones_bool_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;

  double best_lap_time_ = 9999.99;
  double last_lap_time_ = 9999.99;
  int hit_cones_counter_ = 0;
  int lap_counter_ = 0;

  double vx_ = 0.0;
  double vy_ = 0.0;
  double r_ = 0.0;
  double ax_ = 0.0;
  double ay_ = 0.0;
  double delta_ = 0.0;

  double bar_size_;
  double max_torque_value_ = 21*12.48;
  double scale_factor_;
  double center_y_;

  double rviz_width_;
  double rviz_height_;
  double rviz_size_;
  
  double grid_margin_;
  double graph_grid_width_;
  double pen_size_;

  double min_vx_ = 0.0;
  double max_vx_ = 15.0;

  double target_speed_;

  double gg_zoom_factor_ = 1.0;
  double gg_center_x_ = 0.0;
  double gg_center_y_ = 0.0;
  bool timer_gg_started_ = false;


  bool eventFilter(QObject* obj, QEvent* event) override;

  //Loginfo colors
  const std::string red = "\033[1;31m";
  const std::string green = "\033[1;32m";
  const std::string yellow = "\033[1;33m";
  const std::string blue = "\033[1;34m";
  const std::string magenta = "\033[1;35m";
  const std::string cyan = "\033[1;36m";
  const std::string reset = "\033[0m";
};
}  // namespace plot_interface