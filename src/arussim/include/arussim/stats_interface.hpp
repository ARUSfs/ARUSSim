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
#include <QTimer>
#include <QPointF>


#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "arussim_msgs/msg/state.hpp"
#include "common_msgs/msg/slam_stats.hpp"
#include "common_msgs/msg/state.hpp"

namespace stats_interface
{
class StatsInterface : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit StatsInterface(QWidget * parent = 0);
  ~StatsInterface() override;

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
  QVector<QPair<double, double>> optimization_time_history_;
  QVector<QPair<double, double>> target_speed_history_;
  QLabel* slam_graph_label_ = nullptr;
  QLabel* slam_jump_label_ = nullptr;
  QLabel* slam_jump_value_label_ = nullptr;       // instantaneous precision (jump)
  QLabel* slam_accuracy_label_ = nullptr;         // instantaneous accuracy
  QLabel* slam_precision_mean_label_ = nullptr;   // mean precision over the window
  QLabel* slam_accuracy_mean_label_ = nullptr;    // mean accuracy over the window

  QLabel* active_vertices_label_;
  QLabel* active_edges_label_;
  QLabel* observed_landmarks_label_;
  QLabel* unmatched_landmarks_label_;
  QLabel* optimization_time_label_;
  QLabel* data_association_time_label_;

  QSize screen_size_;

  QComboBox* circuit_select_ = nullptr;
  QComboBox* launch_select_ = nullptr;

  QPoint gg_last_mouse_pos_;

  QLabel* gg_legend_label_;

private Q_SLOTS:
  void launch_button_clicked();
  void reset_callback();
  void update_optimizer_time_graph();
  void update_telemetry_labels();
  void stats_callback(double active_vertices, double active_edges, double observed_landmarks, double unmatched_landmarks,double optimization_time, double data_association_time);
  void zoom_in_slam_graph();
  void zoom_out_slam_graph();
  void zoom_in_gg_graph();
  void zoom_out_gg_graph();
  void update_slam_jump_graph();
  void state_callback(const common_msgs::msg::State::SharedPtr msg);
  void render_tick();
  void collect_optimizer_sample();
  void rebuild_jump_background(int w, int h);
  void rebuild_optimizer_background(int w, int h);
  QVector<QPointF> decimate_series(const QVector<QPair<double, double>>& hist,
                                   double current_time, double window,
                                   int w, int h, double vmin, double vrange);
  double mean_of(const QVector<QPair<double, double>>& hist);
  bool interpolate_gt(double t, double& gx, double& gy);

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr torque_sub_;
  rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<arussim_msgs::msg::State>::SharedPtr gt_state_sub_;
  rclcpp::Subscription<common_msgs::msg::SlamStats>::SharedPtr stats_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hit_cones_bool_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;

  double best_lap_time_ = 9999.99;
  double last_lap_time_ = 9999.99;
  int hit_cones_counter_ = 0;
  int lap_counter_ = 0;

  double active_vertices_ = 0.0;
  double active_edges_ = 0.0;
  double observed_landmarks_ = 0.0;
  double unmatched_landmarks_ = 0.0;
  double optimization_time_ = 0.0;
  double data_association_time_ = 0.0;

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

  double min_optimization_time_ = 0.0;
  double max_optimization_time_ = 15.0;

  double target_speed_;

  double gg_zoom_factor_ = 1.0;
  double gg_center_x_ = 0.0;
  double gg_center_y_ = 0.0;
  bool timer_gg_started_ = false;

  // State history for slam jump detection
  QVector<QPair<double, double>> slam_jump_history_;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  double prev_yaw_ = 0.0;
  double prev_vx_ = 0.0;
  double prev_vy_ = 0.0;
  double prev_stamp_ = 0.0;  // previous message timestamp (s), used for dt
  bool has_prev_state_ = false;
  double slam_jump_ = 0.0;
  double min_slam_jump_ = 0.0;
  double max_slam_jump_ = 0.1;

  // SLAM accuracy: pose error of the estimate (/car_state/state) against the
  // simulator ground truth (/arussim/state). Each SLAM state is matched to the
  // ground-truth sample with the nearest header timestamp, so the two streams
  // are associated in time rather than "latest received". Shares the jump
  // graph's y-scale (max_slam_jump_/min_slam_jump_ span both series).
  struct GtSample { double stamp; double x; double y; double yaw; };
  QVector<GtSample> gt_buffer_;
  static constexpr int kGtBufferMax = 512;      // bound the ground-truth buffer
  static constexpr double kGtMatchTol = 0.1;    // s; skip if no GT this close in time
  QVector<QPair<double, double>> slam_accuracy_history_;
  double slam_accuracy_ = 0.0;

  // Render throttling: repaint at a fixed frame rate instead of once per message
  static constexpr int kRenderIntervalMs = 100;  // ~20 fps
  QTimer* render_timer_ = nullptr;
  bool slam_jump_dirty_ = false;
  bool optimizer_dirty_ = false;
  bool telemetry_dirty_ = false;

  // Cached static backgrounds (grid + numeric scale). Rebuilt only when the
  // widget size or the value range changes, so drawText is not run every frame;
  // each frame just copies the background and draws the dynamic data line.
  QPixmap jump_bg_;
  double jump_bg_min_ = 1.0;    // sentinels that force a first rebuild
  double jump_bg_max_ = -1.0;
  QPixmap opt_bg_;
  double opt_bg_max_ = -1.0;

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
}  // namespace stats_interface