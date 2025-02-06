#include <arussim/extended_interface.hpp>

namespace rviz_panel_tutorial
{

ExtendedInterface::ExtendedInterface(QWidget* parent) : Panel(parent)
{
  QFont custom_font_("Montserrat Regular", 13);
  QFont bold_font_ = custom_font_;
  bold_font_.setBold(true);

  // Main vertical layout
  auto main_layout = new QVBoxLayout(this);

  // Create a grid layout for the labels with vertical and horizontal separators
  auto grid_layout = new QGridLayout();
  grid_layout->setContentsMargins(10,10,10,10);
  grid_layout->setSpacing(10);
  grid_layout->setAlignment(Qt::AlignTop);

  // Row 0: First row labels with vertical separator
  last_lt_label_ = new QLabel("Last Lap Time: 0s", this);
  last_lt_label_->setFont(bold_font_);
  grid_layout->addWidget(last_lt_label_, 0, 0);

  auto separator1 = new QFrame(this);
  separator1->setFrameShape(QFrame::VLine);
  separator1->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(separator1, 0, 1);

  lap_label_ = new QLabel("Lap: 0", this);
  lap_label_->setFont(bold_font_);
  grid_layout->addWidget(lap_label_, 0, 2);

  // Row 1: Horizontal separator spanning all columns
  auto h_separator = new QFrame(this);
  h_separator->setFrameShape(QFrame::HLine);
  h_separator->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(h_separator, 1, 0, 1, 3); // span 3 columns

  // Row 2: Second row labels with vertical separator
  best_lt_label_ = new QLabel("Best Lap Time: 0s", this);
  best_lt_label_->setFont(bold_font_);
  grid_layout->addWidget(best_lt_label_, 2, 0);

  auto separator2 = new QFrame(this);
  separator2->setFrameShape(QFrame::VLine);
  separator2->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(separator2, 2, 1);

  hit_cones_label_ = new QLabel("Hit cones: 0", this);
  hit_cones_label_->setFont(bold_font_);
  grid_layout->addWidget(hit_cones_label_, 2, 2);

  // Add the grid layout at the top of the main layout
  main_layout->addLayout(grid_layout);

  // Launch button
  launch_button_ = new QPushButton("Launch Simulation", this);
  main_layout->addWidget(launch_button_);
  connect(launch_button_, &QPushButton::clicked, this, &ExtendedInterface::launch_button_clicked);

  // Stop button
  stop_button_ = new QPushButton("Stop Simulation", this);
  main_layout->addWidget(stop_button_);
  connect(stop_button_, &QPushButton::clicked, this, &ExtendedInterface::stop_button_clicked);

  // Reset button
  reset_button_ = new QPushButton("Reset Simulation", this);
  main_layout->addWidget(reset_button_);
  connect(reset_button_, &QPushButton::clicked, this, &ExtendedInterface::reset_button_clicked);
}

ExtendedInterface::~ExtendedInterface() = default;

void ExtendedInterface::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  // Create reset publisher using the ROS node
  reset_pub_ = node->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);

  // Subscriber
  // torque_sub_ = this->create_subscription<arussim_msgs::msg::FourWheelDrive>(
  //     "/arussim/torque4WD", 1, 
  //     [this](const arussim_msgs::msg::FourWheelDrive::SharedPtr msg) { 
  //         QMetaObject::invokeMethod(this, [this, msg]() {
  //             update_telemetry_bar(msg->front_right, msg->front_left, msg->rear_right, msg->rear_left);
  //         }, Qt::QueuedConnection);
  //     }
  // );

  // state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
  //     "/arussim/state", 1, 
  //     [this](const arussim_msgs::msg::State::SharedPtr msg) { 
  //         QMetaObject::invokeMethod(this, [this, msg]() {
  //             update_telemetry_labels(msg->vx, msg->vy, msg->r, msg->ax, msg->ay, msg->delta);
  //         }, Qt::QueuedConnection);
  //     }
  // );

  lap_time_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/arussim/lap_time", 1, 
      [this](const std_msgs::msg::Float32::SharedPtr msg) { 
          QMetaObject::invokeMethod(this, [this, msg]() {
              update_lap_time_labels(msg->data);
          }, Qt::QueuedConnection);
      }
  );

  hit_cones_bool_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/arussim/hit_cones_bool", 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
              QMetaObject::invokeMethod(this, [this]() {
                  hit_cones_counter_++;
                  hit_cones_label_->setText("Hit cones: " + QString::number(hit_cones_counter_));
              }, Qt::QueuedConnection);
          }
      }
  );
}

/**
 * @brief Update the lap time labels
 * 
 * @param time_list_ 
 */
void ExtendedInterface::update_lap_time_labels(double lap_time_)
{
    lap_counter_++;
    lap_label_->setText("Lap: " + QString::number(lap_counter_));

    if (lap_time_ < best_lap_time_) {
        best_lap_time_ = lap_time_;
        best_lt_label_->setText("Best Lap Time: " + QString::number(best_lap_time_, 'f', 3) + "s");
        last_lt_label_->setStyleSheet("color: purple;");
    } else if (lap_time_ < last_lap_time_) {
        last_lap_time_ = lap_time_;
        last_lt_label_->setStyleSheet("color: green;");
    } else {
        last_lt_label_->setStyleSheet("color: black;");
    }
    last_lt_label_->setText("Last Lap Time: " + QString::number(lap_time_, 'f', 3) + "s");
}

/**
 * @brief Callback for the launch button
 * 
 */
void ExtendedInterface::launch_button_clicked()
{
    simulation_process_ = new QProcess(this);
    QStringList arguments;
    arguments << "launch" << "common_meta" << "simulation_launch.py";
    simulation_process_->start("ros2", arguments);
}

/**
 * @brief Callback for the stop button
 * 
 */
void ExtendedInterface::stop_button_clicked()
{
    if (simulation_process_) {
        kill(static_cast<pid_t>(simulation_process_->processId()), SIGINT);
        simulation_process_->waitForFinished();
        simulation_process_->deleteLater();
        simulation_process_ = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("ExtendedInterface"), "%sSimulation stopped%s", red.c_str(), reset.c_str());
    }
}

/**
 * @brief Callback for the reset button
 * 
 */
void ExtendedInterface::reset_button_clicked()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    reset_pub_->publish(msg);

    best_lap_time_ = 9999.99;
    last_lap_time_ = 9999.99;
    hit_cones_counter_ = 0;
    lap_counter_ = 0;

    best_lt_label_->setText("Best Lap Time: 0s");
    last_lt_label_->setText("Last Lap Time: 0s");
    hit_cones_label_->setText("Hit cones: 0");
    lap_label_->setText("Lap: 0");

    RCLCPP_INFO(rclcpp::get_logger("ExtendedInterface"), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::ExtendedInterface, rviz_common::Panel)