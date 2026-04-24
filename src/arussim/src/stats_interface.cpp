#include <arussim/stats_interface.hpp>

namespace stats_interface
{

StatsInterface::StatsInterface(QWidget* parent) : Panel(parent)
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
    pen_size_ = rviz_size_ * 0.001;

    // Main vertical layout
    auto main_layout = new QVBoxLayout(this);

    // Main grid layout
    auto main_grid = new QGridLayout();
    main_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    main_grid->setSpacing(grid_margin_);
    main_grid->setAlignment(Qt::AlignTop);

    auto graph_grid = new QGridLayout();
    graph_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    graph_grid->setSpacing(grid_margin_);
    graph_grid->setAlignment(Qt::AlignTop);

    // slam graph container
    auto slam_graph_container = new QWidget(this);
    auto slam_graph_layout = new QVBoxLayout(slam_graph_container);
    slam_graph_layout->setContentsMargins(0, 0, 0, 0);
    slam_graph_layout->setSpacing(0);

    slam_graph_label_ = new QLabel(this);
    slam_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
    slam_graph_label_->setFixedHeight(rviz_height_ * 0.15);
    slam_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    slam_graph_label_->setStyleSheet("border: 2px solid black;");
    slam_graph_layout->addWidget(slam_graph_label_);

    auto slam_zoom_layout = new QHBoxLayout();
    auto slam_zoom_out_button = new QPushButton("+", this);
    auto slam_zoom_in_button = new QPushButton("-", this);
    connect(slam_zoom_in_button, &QPushButton::clicked, this, &StatsInterface::zoom_in_slam_graph);
    connect(slam_zoom_out_button, &QPushButton::clicked, this, &StatsInterface::zoom_out_slam_graph);
    slam_zoom_layout->addWidget(slam_zoom_out_button);
    slam_zoom_layout->addWidget(slam_zoom_in_button);
    slam_graph_layout->addLayout(slam_zoom_layout);

    graph_grid->addWidget(slam_graph_container, 0, 0);

    main_grid->addLayout(graph_grid, 1, 0);


    auto telemetry_labels_grid = new QGridLayout();
    telemetry_labels_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    telemetry_labels_grid->setSpacing(grid_margin_);
    telemetry_labels_grid->setAlignment(Qt::AlignTop);

    optimization_time_label_ = new QLabel("Optimization Time: 0", this);
    QFont font = optimization_time_label_->font();
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    optimization_time_label_->setFont(font);
    telemetry_labels_grid->addWidget(optimization_time_label_, 0, 0);

    main_grid->addLayout(telemetry_labels_grid, 3, 0);

    // Plot timer
    timer_.start();

    auto scroll_area = new QScrollArea(this);
    scroll_area->setWidgetResizable(true);
    auto container_widget = new QWidget();
    container_widget->setLayout(main_grid);
    scroll_area->setWidget(container_widget);
    main_layout->addWidget(scroll_area);
}

StatsInterface::~StatsInterface() = default;

void StatsInterface::onInitialize()
{
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

    // Subscribers
    stats_sub_ = node->create_subscription<common_msgs::msg::SlamStats>(
        "/slam/stats", 1, 
        [this](const common_msgs::msg::SlamStats::SharedPtr msg) { 
            QMetaObject::invokeMethod(this, [this, msg]() {
                stats_callback(msg->optimization_time);
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

    slam_graph_label_->installEventFilter(this);
}

bool StatsInterface::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == slam_graph_label_) {
        if (event->type() == QEvent::Wheel) {
            auto* wheel_event = static_cast<QWheelEvent*>(event);
            double angle_delta = wheel_event->angleDelta().y();
            // Accumulate delta until sensibility param is reached
            double accumulated_delta = 0.0;
            accumulated_delta += angle_delta;
            const double sensibility = 30.0;
            if (accumulated_delta >= sensibility) {
                max_optimization_time_ += 5.0;
                accumulated_delta = 0.0;
            } else if (accumulated_delta <= -sensibility) {
                max_optimization_time_ -= 5.0;
                accumulated_delta = 0.0;
            }
        }
    }
    return QObject::eventFilter(obj, event);
}

void StatsInterface::zoom_in_slam_graph()
{
    max_optimization_time_ += 5.0;
}

void StatsInterface::zoom_out_slam_graph()
{
    max_optimization_time_ -= 5.0;
}

void StatsInterface::zoom_in_gg_graph()
{
    gg_zoom_factor_ *= 1.1;
}

void StatsInterface::zoom_out_gg_graph()
{
    gg_zoom_factor_ *= 0.9;
}


/**
 * @brief Update the telemetry labels
 * 
 * @param optimization_time_ 
 */
void StatsInterface::stats_callback(double optimization_time)
{
    optimization_time_ = optimization_time;
    update_telemetry_labels();
    update_optimizer_time_graph();
}

void StatsInterface::update_optimizer_time_graph()
{
    // Get the elapsed time in seconds from the start
    double current_time = timer_.elapsed() / 1000.0;
    const double window = 30.0;

    // Add the new data point
    optimization_time_history_.append(qMakePair(current_time, optimization_time_));

    // Calculate max data
    double current_max = 0.025; // Mínimo una división para evitar división por cero
    for (const auto &point : optimization_time_history_) {
        current_max = std::max(current_max, point.second);
    }
    max_optimization_time_ = std::ceil(current_max / 0.025) * 0.025;

    // Remove points older than 60 seconds
    while (!optimization_time_history_.isEmpty() && (current_time - optimization_time_history_.first().first > window))    {
        optimization_time_history_.removeFirst();
        target_speed_history_.removeFirst();
    }

    // Configure plot dimensions
    int pixmap_width = slam_graph_label_->width();
    int pixmap_height = slam_graph_label_->height();
    QPixmap pixmap(pixmap_width, pixmap_height);
    pixmap.fill(Qt::white);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw grid lines
    int num_rows = qRound(max_optimization_time_ / 0.025);
    if (num_rows <= 0) num_rows = 1;

    double step_y = static_cast<double>(pixmap_height) / num_rows;
    painter.setPen(QPen(Qt::lightGray, graph_grid_width_, Qt::DashLine));
    for (int j = 1; j < num_rows; ++j) {
        int y_line = pixmap_height - qRound(j * step_y);
        painter.drawLine(0, y_line, pixmap_width, y_line);
    }

    // Draw numbers 0.025, 0.050, 0.075... at the same height as each row of the grid
    painter.setPen(Qt::black);
    for (int j = 0; j <= num_rows; ++j) {
        double val = j * 0.025;
        int y_pos = pixmap_height - (j * step_y);
        int text_y = (j == num_rows) ? y_pos + 12 : y_pos; 
        painter.drawText(5, text_y, QString::number(val, 'f', 3));
    }

    if(optimization_time_history_.isEmpty()){
        slam_graph_label_->setPixmap(pixmap);
        return;
    }

    // Draw optimization time line in red
    painter.setPen(QPen(Qt::red, pen_size_));
    QPainterPath optimization_time_path;
    bool first_optimization_time_point = true;
    for (const auto &point : optimization_time_history_)
    {
        double x = ((point.first - (current_time - window)) / window) * pixmap_width;
        double norm = (point.second - min_optimization_time_) / (max_optimization_time_ - min_optimization_time_);
        double y = pixmap_height - (norm * pixmap_height);
        if(first_optimization_time_point) {
            optimization_time_path.moveTo(x, y);
            first_optimization_time_point = false;
        } else {
            optimization_time_path.lineTo(x, y);
        }
    }
    painter.drawPath(optimization_time_path);


    // Draw legend
    QString legend_text_optimization_time = tr("Optimization time");
    int legend_width = QFontMetrics(painter.font()).horizontalAdvance(legend_text_optimization_time)*2;
    int legend_height = QFontMetrics(painter.font()).height()*2.5;
    QRect legend_rect(20, 10, legend_width, legend_height);
    painter.setPen(Qt::black);
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.drawRect(legend_rect);
    
    int line_length = legend_width / 4;
    int y_target = legend_rect.top() + legend_height / 3;
    int y_optimization_time = legend_rect.top() + 2 * legend_height / 3;

    // Legend font size
    QFont legend_font = painter.font();
    legend_font.setPointSizeF(std::min(pixmap_height * 0.075, 12.0));
    painter.setFont(legend_font);

    // Legend for optimization time (red)
    painter.setPen(QPen(Qt::red, pen_size_));
    painter.drawLine(legend_rect.left() + 10, y_optimization_time, legend_rect.left() + 10 + line_length, y_optimization_time);
    painter.setPen(Qt::black);
    painter.drawText(legend_rect.left() + 10 + line_length + 10, y_optimization_time + 5, legend_text_optimization_time);

    slam_graph_label_->setPixmap(pixmap);
}

void StatsInterface::update_telemetry_labels()
{
    optimization_time_label_->setText("Optimization Time: " + QString::number(optimization_time_, 'f', 5));
}

/**
 * @brief Callback for the reset button
 * 
 */
void StatsInterface::reset_callback()
{
    // reset graphs
    optimization_time_history_.clear();
    slam_graph_label_->clear();
    optimization_time_label_->setText("Optimization Time: " + QString::number(0.0, 'f', 5));
}

}  // namespace stats_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(stats_interface::StatsInterface, rviz_common::Panel)