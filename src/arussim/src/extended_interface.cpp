#include <arussim/extended_interface.hpp>

namespace rviz_panel_tutorial
{

ExtendedInterface::ExtendedInterface(QWidget* parent) : Panel(parent)
{
  // Detect screen size to adapt every item to the screen
  QScreen* screen = this->screen();
  screen_size_ = screen->size();

  rviz_width_ = screen_size_.width();
  rviz_height_ = screen_size_.height();
  rviz_size_ = std::sqrt(rviz_width_ * rviz_width_ + rviz_height_ * rviz_height_);

  bar_size_ = rviz_height_ * 0.07;
  scale_factor_ = bar_size_ / max_torque_value_;
  center_y_ = bar_size_ / 2;

  grid_margin_ = rviz_size_ * 0.003;
  graph_grid_width_ = rviz_size_ * 0.001;
  pen_size_ = rviz_size_ * 0.0015;

  // Main vertical layout
  auto main_layout = new QVBoxLayout(this);

  // Main grid layout
  auto main_grid = new QGridLayout();
  main_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  main_grid->setSpacing(grid_margin_);
  main_grid->setAlignment(Qt::AlignTop);

  // Create a grid layout for the labels with vertical and horizontal separators
  auto grid_layout = new QGridLayout();
  grid_layout->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  grid_layout->setSpacing(grid_margin_);
  grid_layout->setAlignment(Qt::AlignTop);

  // Row 0: First row labels with vertical separator
  last_lt_label_ = new QLabel("Last Lap Time: 0s", this);
  grid_layout->addWidget(last_lt_label_, 0, 0);

  auto separator1 = new QFrame(this);
  separator1->setFrameShape(QFrame::VLine);
  separator1->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(separator1, 0, 1);

  lap_label_ = new QLabel("Lap: 0", this);
  grid_layout->addWidget(lap_label_, 0, 2);

  // Row 1: Horizontal separator spanning all columns
  auto h_separator = new QFrame(this);
  h_separator->setFrameShape(QFrame::HLine);
  h_separator->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(h_separator, 1, 0, 1, 3); // span 3 columns

  // Row 2: Second row labels with vertical separator
  best_lt_label_ = new QLabel("Best Lap Time: 0s", this);
  grid_layout->addWidget(best_lt_label_, 2, 0);

  auto separator2 = new QFrame(this);
  separator2->setFrameShape(QFrame::VLine);
  separator2->setFrameShadow(QFrame::Sunken);
  grid_layout->addWidget(separator2, 2, 1);

  hit_cones_label_ = new QLabel("Hit cones: 0", this);
  grid_layout->addWidget(hit_cones_label_, 2, 2);

  // Add the grid layout at the top of the main layout
  main_grid->addLayout(grid_layout, 0, 0);

  // Front Left container and bar without extra block
  telemetry_container_fl_ = new QWidget(this);
  telemetry_container_fl_->setStyleSheet("background-color: lightgray;");
  telemetry_container_fl_->setMinimumWidth(bar_size_);
  telemetry_container_fl_->setFixedHeight(bar_size_);
  telemetry_container_fl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_fl_ = new QWidget(telemetry_container_fl_);
  telemetry_bar_fl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_fl_->move(0, center_y_);
  telemetry_bar_fl_->setFixedWidth(telemetry_container_fl_->width());

  // Front Right container and bar
  telemetry_container_fr_ = new QWidget(this);
  telemetry_container_fr_->setStyleSheet("background-color: lightgray;");
  telemetry_container_fr_->setMinimumWidth(bar_size_);
  telemetry_container_fr_->setFixedHeight(bar_size_);
  telemetry_container_fr_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_fr_ = new QWidget(telemetry_container_fr_);
  telemetry_bar_fr_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_fr_->move(0, center_y_);
  telemetry_bar_fr_->setFixedWidth(telemetry_container_fr_->width());

  // Rear Left container and bar
  telemetry_container_rl_ = new QWidget(this);
  telemetry_container_rl_->setStyleSheet("background-color: lightgray;");
  telemetry_container_rl_->setMinimumWidth(bar_size_);
  telemetry_container_rl_->setFixedHeight(bar_size_);
  telemetry_container_rl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_rl_ = new QWidget(telemetry_container_rl_);
  telemetry_bar_rl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_rl_->move(0, center_y_);
  telemetry_bar_rl_->setFixedWidth(telemetry_container_rl_->width());

  // Rear Right container and bar
  telemetry_container_rr_ = new QWidget(this);
  telemetry_container_rr_->setStyleSheet("background-color: lightgray;");
  telemetry_container_rr_->setMinimumWidth(bar_size_);
  telemetry_container_rr_->setFixedHeight(bar_size_);
  telemetry_container_rr_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_rr_ = new QWidget(telemetry_container_rr_);
  telemetry_bar_rr_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  telemetry_bar_rr_->move(0, center_y_);
  telemetry_bar_rr_->setFixedWidth(telemetry_container_rr_->width());

  // Create telemetry grid and add containers
  auto telemetry_grid = new QGridLayout();
  telemetry_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  telemetry_grid->setSpacing(grid_margin_);
  telemetry_grid->setAlignment(Qt::AlignTop);
  telemetry_grid->addWidget(telemetry_container_fl_, 0, 0);
  telemetry_grid->addWidget(telemetry_container_fr_, 0, 1);
  telemetry_grid->addWidget(telemetry_container_rl_, 1, 0);
  telemetry_grid->addWidget(telemetry_container_rr_, 1, 1);
  main_grid->addLayout(telemetry_grid, 1, 0);

  auto graph_grid = new QGridLayout();
  graph_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  graph_grid->setSpacing(grid_margin_);
  graph_grid->setAlignment(Qt::AlignTop);

  speed_graph_label_ = new QLabel(this);
  speed_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
  speed_graph_label_->setFixedHeight(rviz_height_ * 0.14);
  speed_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  speed_graph_label_->setStyleSheet("border: 2px solid black;");
  graph_grid->addWidget(speed_graph_label_, 0, 0);

  gg_graph_label_ = new QLabel(this);
  gg_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
  gg_graph_label_->setFixedHeight(rviz_height_ * 0.14);
  gg_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  gg_graph_label_->setStyleSheet("border: 2px solid black;");
  graph_grid->addWidget(gg_graph_label_, 1, 0);

  auto telemetry_labels_grid = new QGridLayout();
  telemetry_labels_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  telemetry_labels_grid->setSpacing(grid_margin_);
  telemetry_labels_grid->setAlignment(Qt::AlignTop);

    vx_label_ = new QLabel("Vx: 0", this);
    QFont font = vx_label_->font();
    font.setPointSize(std::min(rviz_height_ * 0.015, 12.0));
    vx_label_->setFont(font);
    telemetry_labels_grid->addWidget(vx_label_, 0, 0);

    vy_label_ = new QLabel("Vy: 0", this);
    vy_label_->setFont(font);
    telemetry_labels_grid->addWidget(vy_label_, 0, 1);

    ax_label_ = new QLabel("Ax: 0", this);
    ax_label_->setFont(font);
    telemetry_labels_grid->addWidget(ax_label_, 1, 0);

    ay_label_ = new QLabel("Ay: 0", this);
    ay_label_->setFont(font);
    telemetry_labels_grid->addWidget(ay_label_, 1, 1);

    r_label_ = new QLabel("r: 0", this);
    r_label_->setFont(font);
    telemetry_labels_grid->addWidget(r_label_, 2, 0);

    delta_label_ = new QLabel("Delta: 0", this);
    delta_label_->setFont(font);
    telemetry_labels_grid->addWidget(delta_label_, 2, 1);

  graph_grid->addLayout(telemetry_labels_grid, 2, 0);


  // Plot timer
  timer_.start();

  main_grid->addLayout(graph_grid, 2, 0);

  // Buttons
  auto button_grid = new QGridLayout();
  button_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
  button_grid->setSpacing(grid_margin_);
  button_grid->setAlignment(Qt::AlignTop);

  // Launch button
  launch_button_ = new QPushButton("Launch Simulation", this);
  launch_button_->setFixedHeight(rviz_height_ * 0.025);
  main_layout->addWidget(launch_button_);
  connect(launch_button_, &QPushButton::clicked, this, &ExtendedInterface::launch_button_clicked);
  button_grid->addWidget(launch_button_, 1, 0);

  // Stop button
  stop_button_ = new QPushButton("Stop Simulation", this);
  stop_button_->setFixedHeight(rviz_height_ * 0.025);
  main_layout->addWidget(stop_button_);
  connect(stop_button_, &QPushButton::clicked, this, &ExtendedInterface::stop_button_clicked);
  button_grid->addWidget(stop_button_, 2, 0);

  // Reset button
  reset_button_ = new QPushButton("Reset Simulation", this);
  reset_button_->setFixedHeight(rviz_height_ * 0.025);
  main_layout->addWidget(reset_button_);
  connect(reset_button_, &QPushButton::clicked, this, &ExtendedInterface::reset_button_clicked);
  button_grid->addWidget(reset_button_, 2, 1);

  // Circuit selection
  circuit_select_ = new QComboBox(this);
  circuit_select_->setFixedHeight(rviz_height_ * 0.025);
  circuit_select_->setPlaceholderText("Choose a circuit");
  std::string package_path_arussim = ament_index_cpp::get_package_share_directory("arussim");
  QString tracks_path = QString::fromStdString(package_path_arussim + "/resources/tracks/");
  QDir tracks_dir(tracks_path);
  QStringList pcd_files = tracks_dir.entryList(QStringList() << "*.pcd", QDir::Files);
  circuit_select_->addItems(pcd_files);
  connect(circuit_select_, &QComboBox::currentTextChanged, this, &ExtendedInterface::circuit_selector);
  button_grid->addWidget(circuit_select_, 1, 1);

  // Launch selection
  launch_select_ = new QComboBox(this);
  launch_select_->setFixedHeight(rviz_height_ * 0.025);
  std::string package_path_launch = ament_index_cpp::get_package_share_directory("common_meta");
  QString launch_path = QString::fromStdString(package_path_launch + "/launch/");
  QDir launch_dir(launch_path);
  QStringList launch_files = launch_dir.entryList(QStringList() << "*.py", QDir::Files);
  launch_select_->addItems(launch_files);
  launch_select_->setCurrentText("simulation_launch.py");
  button_grid->addWidget(launch_select_, 0, 0, 1, 2);

  main_grid->addLayout(button_grid, 3, 0);

  main_layout->addLayout(main_grid);
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

  // Publishers
  reset_pub_ = node->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);
  circuit_pub_ = node->create_publisher<std_msgs::msg::String>("/arussim/circuit", 1);


  // Subscribers
  torque_sub_ = node->create_subscription<arussim_msgs::msg::FourWheelDrive>(
      "/arussim/torque4WD", 1, 
      [this](const arussim_msgs::msg::FourWheelDrive::SharedPtr msg) { 
          QMetaObject::invokeMethod(this, [this, msg]() {
              update_telemetry_bar(msg->front_right, msg->front_left, msg->rear_right, msg->rear_left);
          }, Qt::QueuedConnection);
      }
  );

  state_sub_ = node->create_subscription<arussim_msgs::msg::State>(
      "/arussim/state", 1, 
      [this](const arussim_msgs::msg::State::SharedPtr msg) { 
          QMetaObject::invokeMethod(this, [this, msg]() {
            state_callback(msg->vx, msg->vy, msg->r, msg->ax, msg->ay, msg->delta);
          }, Qt::QueuedConnection);
      }
  );

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

  target_speed_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/controller/target_speed", 1,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
          QMetaObject::invokeMethod(this, [this, msg]() {
              target_speed_ = msg->data;
          }, Qt::QueuedConnection);
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
 * @brief Update the telemetry bars
 * 
 * @param fl_param_ 
 * @param fr_param_ 
 * @param rl_param_ 
 * @param rr_param_ 
 */
void ExtendedInterface::update_telemetry_bar(double fr_param_, double fl_param_, double rr_param_, double rl_param_)
{
    // Front Left
    double height_fl = std::abs(fl_param_) * scale_factor_;
    height_fl = std::min(height_fl, static_cast<double>(telemetry_container_fl_->height()));
    telemetry_bar_fl_->setFixedHeight(static_cast<int>(height_fl));
    telemetry_bar_fl_->setFixedWidth(telemetry_container_fl_->width());
    if (fl_param_ >= 0) {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), center_y_ - height_fl);
        telemetry_bar_fl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), center_y_);
        telemetry_bar_fl_->setStyleSheet("background-color: red;");
    }

    // Front Right
    double height_fr = std::abs(fr_param_) * scale_factor_;
    height_fr = std::min(height_fr, static_cast<double>(telemetry_container_fl_->height()));
    telemetry_bar_fr_->setFixedHeight(static_cast<int>(height_fr));
    telemetry_bar_fr_->setFixedWidth(telemetry_container_fr_->width());
    if (fr_param_ >= 0) {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), center_y_ - height_fr);
        telemetry_bar_fr_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), center_y_);
        telemetry_bar_fr_->setStyleSheet("background-color: red;");
    }

    // Rear Left
    double height_rl = std::abs(rl_param_) * scale_factor_;
    height_rl = std::min(height_rl, static_cast<double>(telemetry_container_fl_->height()));
    telemetry_bar_rl_->setFixedHeight(static_cast<int>(height_rl));
    telemetry_bar_rl_->setFixedWidth(telemetry_container_rl_->width());
    if (rl_param_ >= 0) {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), center_y_ - height_rl);
        telemetry_bar_rl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), center_y_);
        telemetry_bar_rl_->setStyleSheet("background-color: red;");
    }

    // Rear Right
    double height_rr = std::abs(rr_param_) * scale_factor_;
    height_rr = std::min(height_rr, static_cast<double>(telemetry_container_fl_->height()));
    telemetry_bar_rr_->setFixedHeight(static_cast<int>(height_rr));
    telemetry_bar_rr_->setFixedWidth(telemetry_container_rr_->width());
    if (rr_param_ >= 0) {
        telemetry_bar_rr_->move(telemetry_bar_rr_->x(), center_y_ - height_rr);
        telemetry_bar_rr_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_rr_->move(telemetry_bar_rr_->x(), center_y_);
        telemetry_bar_rr_->setStyleSheet("background-color: red;");
    }
}

/**
 * @brief Update the telemetry labels
 * 
 * @param vx_ 
 * @param vy_ 
 * @param r_ 
 * @param ax_ 
 * @param ay_ 
 * @param delta_ 
 */
void ExtendedInterface::state_callback(double vx_, double vy_, double r_, double ax_, double ay_, double delta_)
{
    update_vx_target_graph(vx_, vy_);
    update_gg_graph(ax_, ay_);
    update_telemetry_labels(vx_, vy_, r_, ax_, ay_, delta_);
}

void ExtendedInterface::update_vx_target_graph(double vx, double vy)
{
  // Get the elapsed time in seconds from the start
  double current_time = timer_.elapsed() / 1000.0;

  // Add the new data point
  vx_history_.append(qMakePair(current_time, vx));
  target_speed_history_.append(qMakePair(current_time, target_speed_));

  // Remove points older than 10 seconds
  while (!vx_history_.isEmpty() && (current_time - vx_history_.first().first > 10.0) && (current_time - target_speed_history_.first().first > 10.0))
  {
    vx_history_.removeFirst();
    target_speed_history_.removeFirst();
  }

  // Configure plot dimensions
  int pixmap_width = speed_graph_label_->width();
  int pixmap_height = speed_graph_label_->height();
  QPixmap pixmap(pixmap_width, pixmap_height);
  pixmap.fill(Qt::white);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw axes
  painter.setPen(Qt::black);
  painter.drawLine(0, pixmap_height-1, pixmap_width, pixmap_height-1); // x-axis
  painter.drawLine(0, 0, 0, pixmap_height); // y-axis

  // Draw grid lines
  int num_rows = 4;
  double step_y = pixmap_height / static_cast<double>(num_rows);
  painter.setPen(QPen(Qt::lightGray, graph_grid_width_, Qt::DashLine));
  for (int j = 1; j < num_rows; ++j) {
      painter.drawLine(0, j * step_y, pixmap_width, j * step_y);
  }

  // Draw numbers 5, 10, 15, 20 at the same height as each row of the grid
  painter.setPen(Qt::black);
  for (int j = 1; j <= num_rows; ++j) {
      painter.drawText(0, pixmap_height - j * step_y, QString::number(j * 5));
  }

  if(vx_history_.isEmpty()){
      speed_graph_label_->setPixmap(pixmap);
      return;
  }

  // Draw target speed line in blue
  painter.setPen(QPen(Qt::blue, pen_size_));
  QPainterPath ts_path;
  bool first_ts_point = true;
  for (const auto &point : target_speed_history_)
  {
      double x = ((point.first - (current_time - 10.0)) / 10.0) * pixmap_width;
      double norm = (point.second - min_vx_) / (max_vx_ - min_vx_);
      double y = pixmap_height - (norm * pixmap_height);
      if(first_ts_point) {
          ts_path.moveTo(x, y);
          first_ts_point = false;
      } else {
          ts_path.lineTo(x, y);
      }
  }
  painter.drawPath(ts_path);

  // Draw vx line in red
  painter.setPen(QPen(Qt::red, pen_size_));
  QPainterPath vx_path;
  bool first_vx_point = true;
  for (const auto &point : vx_history_)
  {
      double x = ((point.first - (current_time - 10.0)) / 10.0) * pixmap_width;
      double norm = (point.second - min_vx_) / (max_vx_ - min_vx_);
      double y = pixmap_height - (norm * pixmap_height);
      if(first_vx_point) {
          vx_path.moveTo(x, y);
          first_vx_point = false;
      } else {
          vx_path.lineTo(x, y);
      }
  }
  painter.drawPath(vx_path);

  // draw legend
  // Adaptable legend for speed graph
  int legend_width = std::min(pixmap_width * 0.35, rviz_size_ * 0.15);
  int legend_height = std::min(pixmap_height * 0.225, rviz_size_ * 0.06);
  QRect legend_rect(20, 10, legend_width, legend_height);
  painter.setPen(Qt::black);
  painter.setBrush(QColor(255, 255, 255, 200));
  painter.drawRect(legend_rect);
  
  int line_length = legend_width / 4;
  int y_target = legend_rect.top() + legend_height / 3;
  int y_vx = legend_rect.top() + 2 * legend_height / 3;

  // Legend font size
  QFont legend_font = painter.font();
  legend_font.setPointSizeF(std::min(pixmap_height * 0.075, 12.0));
  painter.setFont(legend_font);
  
  // Legend for Target Speed (blue)
  painter.setPen(QPen(Qt::blue, pen_size_*0.9));
  painter.drawLine(legend_rect.left() + 10, y_target, legend_rect.left() + 10 + line_length, y_target);
  painter.setPen(Qt::black);
  painter.drawText(legend_rect.left() + 10 + line_length + 10, y_target + 5, "Target");
  
  // Legend for Vx (red)
  painter.setPen(QPen(Qt::red, pen_size_));
  painter.drawLine(legend_rect.left() + 10, y_vx, legend_rect.left() + 10 + line_length, y_vx);
  painter.setPen(Qt::black);
  painter.drawText(legend_rect.left() + 10 + line_length + 10, y_vx + 5, "Vx");

  speed_graph_label_->setPixmap(pixmap);
}

void ExtendedInterface::update_gg_graph(double ax, double ay)
{
  gg_vector_.append(qMakePair(ay, ax));

  if (!gg_graph_label_) return;

  int pixmap_width = gg_graph_label_->width();
  int pixmap_height = gg_graph_label_->height();
  QPixmap pixmap(pixmap_width, pixmap_height);
  pixmap.fill(Qt::white);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);

  // main axes with color
  int cx = pixmap_width / 2;
  int cy = pixmap_height / 2;

  // Draw grid for integers from -12 to 12 (skip 0, already drawn by the axis)
  for (int i = -12; i <= 12; i++)
  {
    if(i == 0) continue; // Skip the axis
    if (i % 2 == 0) {
        painter.setPen(QPen(Qt::lightGray, graph_grid_width_));
    } else {
        painter.setPen(QPen(Qt::lightGray, graph_grid_width_/2));
    }
    double x = (i + 12.0) / 24.0 * pixmap_width;
    double y = pixmap_height - ((i + 12.0) / 24.0 * pixmap_height);

    // Vertical line
    painter.drawLine(x, 0, x, pixmap_height);

    // Horizontal line
    painter.drawLine(0, y, pixmap_width, y);
  }

  // Centered axes
  painter.setPen(QPen(Qt::black, graph_grid_width_*2));
  painter.drawLine(0, cy, pixmap_width, cy); // x-axis
  painter.drawLine(cx, 0, cx, pixmap_height);  // y-axis

  // draw GG legend
  int legend_width = std::min(pixmap_width * 0.35, rviz_size_ * 0.15);
  int legend_height = std::min(pixmap_height * 0.15, rviz_size_ * 0.05);
  QRect legend_gg_rect(10, 10, legend_width, legend_height);
  painter.setPen(Qt::black);
  painter.setBrush(QColor(255, 255, 255, 200));
  painter.drawRect(legend_gg_rect);
  QFont legend_font = painter.font();
  legend_font.setPointSizeF(std::min(pixmap_height * 0.075, 12.0));
  painter.setFont(legend_font);
  painter.drawText(legend_gg_rect.adjusted(10, 0, 0, 0), "GG diagram");

  // Draw points from gg_vector_ with new scaling for range -12 to 12
  painter.setPen(QPen(Qt::blue, pen_size_));
  for (auto &p : gg_vector_) {
    double x = (p.first + 12.0) / 24.0 * pixmap_width;
    double y = pixmap_height - ((p.second + 12.0) / 24.0 * pixmap_height);
    painter.drawPoint(QPointF(x, y));
  }

  gg_graph_label_->setPixmap(pixmap);
}

void ExtendedInterface::update_telemetry_labels(double vx, double vy, double r, double ax, double ay, double delta)
{
    vx_label_->setText("Vx: " + QString::number(vx, 'f', 2));
    vy_label_->setText("Vy: " + QString::number(vy, 'f', 2));
    r_label_->setText("r: " + QString::number(r, 'f', 2));
    ax_label_->setText("Ax: " + QString::number(ax, 'f', 2));
    ay_label_->setText("Ay: " + QString::number(ay, 'f', 2));
    delta_label_->setText("Delta: " + QString::number(delta, 'f', 2));
}

/**
 * @brief Callback for the launch button
 * 
 */
void ExtendedInterface::launch_button_clicked()
{
    if (simulation_process_ == nullptr) {
        simulation_process_ = new QProcess(this);
        QString launch_file = launch_select_->currentText();
        QStringList args;
        args << "launch" << "common_meta" << launch_file;
        simulation_process_->start("ros2", args);
    }
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

    // reset graphs
    vx_history_.clear();
    target_speed_history_.clear();
    gg_vector_.clear();
    speed_graph_label_->clear();
    gg_graph_label_->clear();

    RCLCPP_INFO(rclcpp::get_logger("ExtendedInterface"), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

/**
 * @brief Callback for the circuit selector
 * 
 * @param option 
 */
void ExtendedInterface::circuit_selector(const QString & option)
{
  auto msg_circuit = std_msgs::msg::String();
  msg_circuit.data = option.toStdString();
  circuit_pub_->publish(msg_circuit);

  auto msg_reset = std_msgs::msg::Bool();
  msg_reset.data = true;
  reset_pub_->publish(msg_reset);

  // reset graphs when switching circuit
  vx_history_.clear();
  target_speed_history_.clear();
  gg_vector_.clear();
  speed_graph_label_->clear();
  gg_graph_label_->clear();

  RCLCPP_INFO(rclcpp::get_logger("ExtendedInterface"), "%sCircuit selected: %s%s", cyan.c_str(), option.toStdString().c_str(), reset.c_str());

}

}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::ExtendedInterface, rviz_common::Panel)