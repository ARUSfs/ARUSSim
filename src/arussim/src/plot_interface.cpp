#include <arussim/plot_interface.hpp>

namespace plot_interface
{

PlotInterface::PlotInterface(QWidget* parent) : Panel(parent)
{
    // Detect screen size to adapt every item to the screen
    QScreen* screen = this->screen();
    screen_size_ = screen->size();

    rviz_width_ = screen_size_.width();
    rviz_height_ = screen_size_.height();
    rviz_size_ = std::sqrt(rviz_width_ * rviz_width_ + rviz_height_ * rviz_height_);

    bar_size_ = rviz_height_ * 0.065;
    scale_factor_ = bar_size_ / max_torque_value_;
    center_y_ = bar_size_ / 2;

    grid_margin_ = rviz_size_ * 0.003;
    graph_grid_width_ = rviz_size_ * 0.001;
    pen_size_ = rviz_size_ * 0.002;

    // Main vertical layout
    auto main_layout = new QVBoxLayout(this);

    // Main grid layout
    auto main_grid = new QGridLayout();
    main_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    main_grid->setSpacing(grid_margin_);
    main_grid->setAlignment(Qt::AlignTop);

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
    main_grid->addLayout(telemetry_grid, 0, 0);

    auto graph_grid = new QGridLayout();
    graph_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    graph_grid->setSpacing(grid_margin_);
    graph_grid->setAlignment(Qt::AlignTop);

    speed_graph_label_ = new QLabel(this);
    speed_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
    speed_graph_label_->setFixedHeight(rviz_height_ * 0.2);
    speed_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    speed_graph_label_->setStyleSheet("border: 2px solid black;");
    graph_grid->addWidget(speed_graph_label_, 0, 0);

    gg_graph_label_ = new QLabel(this);
    gg_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
    gg_graph_label_->setMinimumHeight(rviz_height_ * 0.2);
    gg_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    gg_graph_label_->setStyleSheet("border: 2px solid black;");
    graph_grid->addWidget(gg_graph_label_, 1, 0);

    main_grid->addLayout(graph_grid, 1, 0);


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

    main_grid->addLayout(telemetry_labels_grid, 3, 0);

    // Plot timer
    timer_.start();

    main_layout->addLayout(main_grid);
}

PlotInterface::~PlotInterface() = default;

void PlotInterface::onInitialize()
{
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

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

    target_speed_sub_ = node->create_subscription<std_msgs::msg::Float32>(
        "/controller/target_speed", 1,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                target_speed_ = msg->data;
            }, Qt::QueuedConnection);
        }
    );

    reset_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "/arussim/reset", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                QMetaObject::invokeMethod(this, [this]() {
                    reset_callback();
                }, Qt::QueuedConnection);
            }
        }
    );

    gg_graph_label_->installEventFilter(this);
}

bool PlotInterface::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == gg_graph_label_) {
        if (event->type() == QEvent::Wheel) {
        auto* wheel_event = static_cast<QWheelEvent*>(event);
        double angleDelta = wheel_event->angleDelta().y();
        gg_zoom_factor_ *= (angleDelta > 0 ? 1.1 : 0.9);
        gg_zoom_factor_ = std::clamp(gg_zoom_factor_, 0.01, 4.0);
        } else if (event->type() == QEvent::MouseButtonPress) {
        auto* mouse_event = static_cast<QMouseEvent*>(event);
        if(mouse_event->button() == Qt::LeftButton)
        {
            gg_last_mouse_pos_ = mouse_event->pos();
        }
        } else if (event->type() == QEvent::MouseMove && QEvent::MouseButtonPress) {
        auto* mouse_event = static_cast<QMouseEvent*>(event);
        // Calculate the position of (0,0) in the graph based on gg_center_x_, gg_center_y_, and gg_zoom_factor_
        QPoint delta = mouse_event->pos() - gg_last_mouse_pos_;
        gg_center_x_ -= delta.x() / (10.0 / gg_zoom_factor_);
        gg_center_y_ += delta.y() / (10.0 / gg_zoom_factor_);
        gg_last_mouse_pos_ = mouse_event->pos();
        }
    }
    return QObject::eventFilter(obj, event);
}

/**
 * @brief Update the telemetry bars
 * 
 * @param fl_param_ 
 * @param fr_param_ 
 * @param rl_param_ 
 * @param rr_param_ 
 */
void PlotInterface::update_telemetry_bar(double fr_param_, double fl_param_, double rr_param_, double rl_param_)
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
void PlotInterface::state_callback(double vx, double vy, double r, double ax, double ay, double delta)
{
    update_vx_target_graph(vx, vy);
    update_gg_graph(ax, ay, vx);
    update_telemetry_labels(vx, vy, r, ax, ay, delta);
}

void PlotInterface::update_vx_target_graph(double vx, double vy)
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

void PlotInterface::update_gg_graph(double ax, double ay, double vx)
{
    // Remove points older than 10 seconds
    if (vx != 0.5){
        gg_vector_.append(qMakePair(ay, ax));
        if (!timer_gg_started_){
            timer_gg_.start();
            timer_gg_started_ = true;
        }
    } if (timer_gg_.elapsed() > 30000) {
        gg_vector_.removeFirst();
    }

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

    // Calculate the position of (0,0) in the graph based on gg_center_x_, gg_center_y_, and gg_zoom_factor_
    double zero_x = ((0.0 - gg_center_x_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_width;
    double zero_y = pixmap_height - ((0.0 - gg_center_y_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_height;

    // Draw the grid applying translation and zoom
    for (int i = -50; i <= 50; i++)
    {
        if(i == 0) continue; 
        double x = ((i - gg_center_x_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_width;
        double y = pixmap_height - ((i - gg_center_y_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_height;
        painter.setPen(QPen(Qt::lightGray, graph_grid_width_));
        painter.drawLine(x, 0, x, pixmap_height);
        painter.drawLine(0, y, pixmap_width, y);
    }

    // Draw axes centered at the real (0,0)
    painter.setPen(QPen(Qt::black, graph_grid_width_*2));
    painter.drawLine(0, zero_y, pixmap_width, zero_y);
    painter.drawLine(zero_x, 0, zero_x, pixmap_height);

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
        double rx = ((p.first - gg_center_x_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_width;
        double ry = pixmap_height - ((p.second - gg_center_y_) + 12.0 * gg_zoom_factor_) / (24.0 * gg_zoom_factor_) * pixmap_height;
        painter.drawPoint(QPointF(rx, ry));
    }

    gg_graph_label_->setPixmap(pixmap);
}

void PlotInterface::update_telemetry_labels(double vx, double vy, double r, double ax, double ay, double delta)
{
    vx_label_->setText("Vx: " + QString::number(vx, 'f', 2));
    vy_label_->setText("Vy: " + QString::number(vy, 'f', 2));
    r_label_->setText("r: " + QString::number(r, 'f', 2));
    ax_label_->setText("Ax: " + QString::number(ax, 'f', 2));
    ay_label_->setText("Ay: " + QString::number(ay, 'f', 2));
    delta_label_->setText("Delta: " + QString::number(delta, 'f', 2));
}

/**
 * @brief Callback for the reset button
 * 
 */
void PlotInterface::reset_callback()
{
    // reset graphs
    vx_history_.clear();
    target_speed_history_.clear();
    gg_vector_.clear();
    speed_graph_label_->clear();
    gg_graph_label_->clear();
}

}  // namespace plot_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(plot_interface::PlotInterface, rviz_common::Panel)