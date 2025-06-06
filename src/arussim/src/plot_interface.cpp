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

    // Speed graph container
    auto speed_graph_container = new QWidget(this);
    auto speed_graph_layout = new QVBoxLayout(speed_graph_container);
    speed_graph_layout->setContentsMargins(0, 0, 0, 0);
    speed_graph_layout->setSpacing(0);

    speed_graph_label_ = new QLabel(this);
    speed_graph_label_->setMinimumWidth(rviz_width_ * 0.1);
    speed_graph_label_->setFixedHeight(rviz_height_ * 0.15);
    speed_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    speed_graph_label_->setStyleSheet("border: 2px solid black;");
    speed_graph_layout->addWidget(speed_graph_label_);

    auto speed_zoom_layout = new QHBoxLayout();
    auto speed_zoom_out_button = new QPushButton("+", this);
    auto speed_zoom_in_button = new QPushButton("-", this);
    connect(speed_zoom_in_button, &QPushButton::clicked, this, &PlotInterface::zoom_in_speed_graph);
    connect(speed_zoom_out_button, &QPushButton::clicked, this, &PlotInterface::zoom_out_speed_graph);
    speed_zoom_layout->addWidget(speed_zoom_out_button);
    speed_zoom_layout->addWidget(speed_zoom_in_button);
    speed_graph_layout->addLayout(speed_zoom_layout);

    graph_grid->addWidget(speed_graph_container, 0, 0);

    // GG graph container
    auto gg_graph_container = new QWidget(this);
    auto gg_graph_layout = new QVBoxLayout(gg_graph_container);
    gg_graph_layout->setContentsMargins(0, 0, 0, 0);
    gg_graph_layout->setSpacing(0);

    gg_graph_label_ = new QLabel(this);
    gg_graph_label_->setMinimumWidth(rviz_width_ * 0.15);
    gg_graph_label_->setMinimumHeight(rviz_height_ * 0.15);
    gg_graph_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    gg_graph_label_->setStyleSheet("border: 2px solid black;");
    gg_graph_layout->addWidget(gg_graph_label_);

    // Add the legend label
    gg_legend_label_ = new QLabel(this);
    gg_legend_label_->setMinimumWidth(rviz_width_ * 0.15);
    gg_legend_label_->setFixedHeight(rviz_height_ * 0.03);
    gg_legend_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    gg_legend_label_->setStyleSheet("border: 2px solid black;");
    gg_graph_layout->addWidget(gg_legend_label_);

    auto gg_zoom_layout = new QHBoxLayout();
    auto gg_zoom_out_button = new QPushButton("+", this);
    auto gg_zoom_in_button = new QPushButton("-", this);
    connect(gg_zoom_in_button, &QPushButton::clicked, this, &PlotInterface::zoom_in_gg_graph);
    connect(gg_zoom_out_button, &QPushButton::clicked, this, &PlotInterface::zoom_out_gg_graph);
    gg_zoom_layout->addWidget(gg_zoom_out_button);
    gg_zoom_layout->addWidget(gg_zoom_in_button);
    gg_graph_layout->addLayout(gg_zoom_layout);

    graph_grid->addWidget(gg_graph_container, 1, 0);

    main_grid->addLayout(graph_grid, 1, 0);


    auto telemetry_labels_grid = new QGridLayout();
    telemetry_labels_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    telemetry_labels_grid->setSpacing(grid_margin_);
    telemetry_labels_grid->setAlignment(Qt::AlignTop);

    vx_label_ = new QLabel("Vx: 0", this);
    QFont font = vx_label_->font();
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
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

    auto scroll_area = new QScrollArea(this);
    scroll_area->setWidgetResizable(true);
    auto container_widget = new QWidget();
    container_widget->setLayout(main_grid);
    scroll_area->setWidget(container_widget);
    main_layout->addWidget(scroll_area);
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

    speed_graph_label_->installEventFilter(this);
    gg_graph_label_->installEventFilter(this);
}

bool PlotInterface::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == gg_graph_label_) {
        if (event->type() == QEvent::Wheel) {
            auto* wheel_event = static_cast<QWheelEvent*>(event);
            double angle_delta = wheel_event->angleDelta().y();
            gg_zoom_factor_ *= (angle_delta > 0 ? 1.1 : 0.9);
            gg_zoom_factor_ = std::clamp(gg_zoom_factor_, 0.01, 4.0);
        } else if (event->type() == QEvent::MouseButtonPress) {
            auto* mouse_event = static_cast<QMouseEvent*>(event);
            if(mouse_event->button() == Qt::LeftButton) {
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
    } else if (obj == speed_graph_label_) {
        if (event->type() == QEvent::Wheel) {
            auto* wheel_event = static_cast<QWheelEvent*>(event);
            double angle_delta = wheel_event->angleDelta().y();
            // Accumulate delta until sensibility param is reached
            double accumulated_delta = 0.0;
            accumulated_delta += angle_delta;
            const double sensibility = 30.0;
            if (accumulated_delta >= sensibility) {
                max_vx_ += 5.0;
                accumulated_delta = 0.0;
            } else if (accumulated_delta <= -sensibility) {
                max_vx_ -= 5.0;
                accumulated_delta = 0.0;
            }
        }
    }
    return QObject::eventFilter(obj, event);
}

void PlotInterface::zoom_in_speed_graph()
{
    max_vx_ += 5.0;
}

void PlotInterface::zoom_out_speed_graph()
{
    max_vx_ -= 5.0;
}

void PlotInterface::zoom_in_gg_graph()
{
    gg_zoom_factor_ *= 1.1;
}

void PlotInterface::zoom_out_gg_graph()
{
    gg_zoom_factor_ *= 0.9;
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

    update_telemetry_labels();
    update_vx_target_graph();
    update_gg_graph();
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
    vx_ = vx;
    vy_ = vy;
    r_ = r;
    ax_ = ax;
    ay_ = ay;
    delta_ = delta;
}

void PlotInterface::update_vx_target_graph()
{
    // Get the elapsed time in seconds from the start
    double current_time = timer_.elapsed() / 1000.0;

    // Add the new data point
    vx_history_.append(qMakePair(current_time, vx_));
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

    // Draw grid lines
    int num_rows = max_vx_ / 5.0;
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

    // Draw legend
    QString legend_text_target = tr("Target");
    QString legend_text_vx = tr("Vx");
    int legend_width = std::max(QFontMetrics(painter.font()).horizontalAdvance(legend_text_target)*2, 
                                QFontMetrics(painter.font()).horizontalAdvance(legend_text_vx)*2);
    int legend_height = QFontMetrics(painter.font()).height()*2.5;
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
    painter.drawText(legend_rect.left() + 10 + line_length + 10, y_target + 5, legend_text_target);
    
    // Legend for Vx (red)
    painter.setPen(QPen(Qt::red, pen_size_));
    painter.drawLine(legend_rect.left() + 10, y_vx, legend_rect.left() + 10 + line_length, y_vx);
    painter.setPen(Qt::black);
    painter.drawText(legend_rect.left() + 10 + line_length + 10, y_vx + 5, legend_text_vx);

    speed_graph_label_->setPixmap(pixmap);
}

void PlotInterface::update_gg_graph()
{
    // Remove points older than 30 seconds
    if (vx_ != 0.5){
        gg_vector_.append(std::make_tuple(ay_, ax_, vx_));
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
    QString legend_text = tr("GG diagram");
    int legend_width = QFontMetrics(painter.font()).horizontalAdvance(legend_text)*1.25;
    int legend_height = QFontMetrics(painter.font()).height()*1.1;
    QRect legend_gg_rect(10, 10, legend_width, legend_height);
    painter.setPen(Qt::black);
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.drawRect(legend_gg_rect);
    QFont legend_font = painter.font();
    legend_font.setPointSizeF(std::min(pixmap_height * 0.075, 10.0));
    painter.setFont(legend_font);
    painter.drawText(legend_gg_rect.adjusted(10, 0, 0, 0), legend_text);

    // Draw points from gg_vector_ with new scaling for range -12 to 12
    for (auto &p : gg_vector_) {
        double loc_ay = std::get<0>(p);
        double loc_ax = std::get<1>(p);
        double loc_vx = std::get<2>(p);

        // Normalize loc_vx using min_vx_ and max_vx_
        double norm = (loc_vx - min_vx_) / (max_vx_ - min_vx_);
        norm = std::clamp(norm, 0.0, 1.0);
        // Map normalized value to hue: 240 (blue) to 0 (red)
        int hue = static_cast<int>(240 * (1 - norm));
        QColor point_color = QColor::fromHsv(hue, 255, 255);

        painter.setPen(QPen(point_color, pen_size_));
        double rx = ((loc_ay - gg_center_x_) + 12.0 * gg_zoom_factor_) 
                    / (24.0 * gg_zoom_factor_) * pixmap_width;
        double ry = pixmap_height - 
                    ((loc_ax - gg_center_y_) + 12.0 * gg_zoom_factor_) 
                    / (24.0 * gg_zoom_factor_) * pixmap_height;
        painter.drawPoint(QPointF(rx, ry));
    }

    gg_graph_label_->setPixmap(pixmap);

    // Draw horizontal multicolor bar legend below the graph
    int legend_bar_height = 20;
    int legend_bar_width = gg_legend_label_->width();
    QPixmap legend_pixmap(legend_bar_width, legend_bar_height + 20);
    legend_pixmap.fill(Qt::white);
    QPainter legend_painter(&legend_pixmap);
    legend_painter.setRenderHint(QPainter::Antialiasing);

    for (int i = 0; i < legend_bar_width; ++i) {
        double norm = static_cast<double>(i) / legend_bar_width;
        int hue = static_cast<int>(240 * (1 - norm));
        QColor color = QColor::fromHsv(hue, 255, 255);
        legend_painter.setPen(QPen(color, 1));
        legend_painter.drawLine(i, 0, i, legend_bar_height);
    }

    // Draw velocity values
    for (double v = min_vx_; v <= max_vx_; v += 2.5) {
        double norm = (v - min_vx_) / (max_vx_ - min_vx_);
        int x = static_cast<int>(norm * legend_bar_width);
        legend_painter.setPen(Qt::black);
        legend_painter.drawText(x, legend_bar_height + 15, QString::number(v, 'f', 1));
    }

    gg_legend_label_->setPixmap(legend_pixmap);
}

void PlotInterface::update_telemetry_labels()
{
    vx_label_->setText("Vx: " + QString::number(vx_, 'f', 2));
    vy_label_->setText("Vy: " + QString::number(vy_, 'f', 2));
    r_label_->setText("r: " + QString::number(r_, 'f', 2));
    ax_label_->setText("Ax: " + QString::number(ax_, 'f', 2));
    ay_label_->setText("Ay: " + QString::number(ay_, 'f', 2));
    delta_label_->setText("Delta: " + QString::number(delta_, 'f', 2));
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
    timer_gg_started_ = false;
}

}  // namespace plot_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(plot_interface::PlotInterface, rviz_common::Panel)