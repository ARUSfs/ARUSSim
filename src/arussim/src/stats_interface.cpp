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

    // SLAM jump graph container (new separate plot)
    auto slam_jump_container = new QWidget(this);
    auto slam_jump_layout = new QVBoxLayout(slam_jump_container);
    slam_jump_layout->setContentsMargins(0, 0, 0, 0);
    slam_jump_layout->setSpacing(0);

    slam_jump_label_ = new QLabel(this);
    slam_jump_label_->setMinimumWidth(rviz_width_ * 0.1);
    slam_jump_label_->setFixedHeight(rviz_height_ * 0.15);
    slam_jump_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    slam_jump_label_->setStyleSheet("border: 2px solid black;");
    slam_jump_layout->addWidget(slam_jump_label_);

    // Place SLAM jump container below the telemetry labels in the main grid
    // (will be added to main_grid after telemetry_labels_grid is created)

    main_grid->addLayout(graph_grid, 1, 0);


    auto telemetry_labels_grid = new QGridLayout();
    telemetry_labels_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    telemetry_labels_grid->setSpacing(grid_margin_);
    telemetry_labels_grid->setAlignment(Qt::AlignTop);

    active_vertices_label_ = new QLabel("Active Vertices: 0", this);
    QFont font = active_vertices_label_->font();
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    active_vertices_label_->setFont(font);
    telemetry_labels_grid->addWidget(active_vertices_label_, 0, 0);

    active_edges_label_ = new QLabel("Active Edges: 0", this);
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    active_edges_label_->setFont(font);
    telemetry_labels_grid->addWidget(active_edges_label_, 1, 0);

    observed_landmarks_label_ = new QLabel("Observed Landmarks: 0", this);
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    observed_landmarks_label_->setFont(font);
    telemetry_labels_grid->addWidget(observed_landmarks_label_, 2, 0);

    unmatched_landmarks_label_ = new QLabel("Unmatched Landmarks: 0", this);
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    unmatched_landmarks_label_->setFont(font);
    telemetry_labels_grid->addWidget(unmatched_landmarks_label_, 3, 0);

    optimization_time_label_ = new QLabel("Optimization Time: 0", this);
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    optimization_time_label_->setFont(font);
    telemetry_labels_grid->addWidget(optimization_time_label_, 4, 0);

    data_association_time_label_ = new QLabel("Data Association Time: 0", this);
    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    data_association_time_label_->setFont(font);
    telemetry_labels_grid->addWidget(data_association_time_label_, 5, 0);

    // Value labels shown below the jump graph: instantaneous precision/accuracy
    // plus their running means over the window. They go in their own sub-layout
    // with the same margins, spacing and font as the telemetry labels above.
    auto slam_values_layout = new QVBoxLayout();
    slam_values_layout->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    slam_values_layout->setSpacing(grid_margin_);
    slam_values_layout->setAlignment(Qt::AlignTop);

    font.setPointSize(std::min(rviz_height_ * 0.015, 10.0));
    auto make_value_label = [&](const QString& text) {
        auto* lbl = new QLabel(text, this);
        lbl->setFont(font);
        slam_values_layout->addWidget(lbl);
        return lbl;
    };
    slam_jump_value_label_     = make_value_label("Slam Jump (Precision): 0");
    slam_accuracy_label_       = make_value_label("Accuracy: 0");
    slam_precision_mean_label_ = make_value_label("Precision (media): 0");
    slam_accuracy_mean_label_  = make_value_label("Accuracy (media): 0");
    slam_jump_layout->addLayout(slam_values_layout);

    main_grid->addLayout(telemetry_labels_grid, 3, 0);

    // Add slam jump container below telemetry labels
    main_grid->addWidget(slam_jump_container, 4, 0);

    // Plot timer
    timer_.start();

    // Render timer: decouple the (expensive) graph repaints from the ROS
    // message rate. Callbacks only collect data and flag the graph as dirty;
    // this timer repaints at a fixed frame rate, so the panel's cost no longer
    // scales with how fast messages arrive.
    render_timer_ = new QTimer(this);
    connect(render_timer_, &QTimer::timeout, this, &StatsInterface::render_tick);
    render_timer_->start(kRenderIntervalMs);

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
                stats_callback(msg->active_vertices, msg->active_edges, msg->observed_landmarks,
                                msg->unmatched_landmarks,msg->optimization_time, msg->data_association_time);
            }, Qt::QueuedConnection);
        }
    );

    // Subscribe to car state to compute slam jumps using member callback
    state_sub_ = node->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 10, std::bind(&StatsInterface::state_callback, this, std::placeholders::_1));

    // Subscribe to the simulator ground-truth pose to compute SLAM accuracy.
    // Buffers each pose with its header timestamp; state_callback then matches
    // every SLAM state to the ground-truth sample nearest in time.
    gt_state_sub_ = node->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 10,
        [this](const arussim_msgs::msg::State::SharedPtr msg) {
            GtSample s{ msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                        msg->x, msg->y, msg->yaw };
            QMetaObject::invokeMethod(this, [this, s]() {
                gt_buffer_.append(s);
                while (gt_buffer_.size() > kGtBufferMax) gt_buffer_.removeFirst();
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

void StatsInterface::state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    // Called from rclcpp thread — move processing to Qt GUI thread
    QMetaObject::invokeMethod(this, [this, msg]() {
        double current_time = timer_.elapsed() / 1000.0;
        double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double dt = 0.0;
        if (has_prev_state_) {
            dt = stamp - prev_stamp_;
            if (dt <= 0.0) dt = 1e-6;
        }

        double x = msg->x;
        double y = msg->y;
        double yaw = msg->yaw;
        double vx = msg->vx;
        double vy = msg->vy;

        // Skip jumps computed across gaps longer than 1 s: over such a long
        // interval the constant-velocity prediction is unreliable and would
        // produce a fake jump. This sample is not plotted, but it still becomes
        // the new baseline below for the following samples.
        if (has_prev_state_ && dt <= 1.0) {
            double obs_dx = x - prev_x_;
            double obs_dy = y - prev_y_;

            // vx/vy are body-frame velocities while x/y are in the global frame,
            // so rotate the predicted displacement by the previous yaw before
            // comparing it against the observed (global) displacement. Otherwise
            // any non-zero heading produces a fake jump of ~v*dt.
            double pred_dx = (prev_vx_ * std::cos(prev_yaw_) - prev_vy_ * std::sin(prev_yaw_)) * dt;
            double pred_dy = (prev_vx_ * std::sin(prev_yaw_) + prev_vy_ * std::cos(prev_yaw_)) * dt;

            double jump_x = obs_dx - pred_dx;
            double jump_y = obs_dy - pred_dy;
            double jump_mag = std::sqrt(jump_x*jump_x + jump_y*jump_y);
            slam_jump_ = jump_mag;

            slam_jump_history_.append(qMakePair(current_time, jump_mag));
            min_slam_jump_ = std::min(min_slam_jump_, jump_mag);
            max_slam_jump_ = std::max(max_slam_jump_, jump_mag);

            const double window = 30.0;
            while (!slam_jump_history_.isEmpty() && (current_time - slam_jump_history_.first().first > window)) {
                slam_jump_history_.removeFirst();
            }

            // Defer the (expensive) repaint to the render timer.
            slam_jump_dirty_ = true;
        }

        // SLAM accuracy: position error (metres) of the estimate (/car_state/state)
        // against the simulator ground truth (/arussim/state). The ground-truth
        // pose is linearly interpolated to this SLAM state's timestamp, so both
        // streams are aligned in time — this removes the speed-dependent error
        // that a nearest-sample match leaves (up to v * half_period). Orientation
        // (yaw) is intentionally NOT mixed in: it is a different unit (rad vs m)
        // and would distort a metric that should read as a distance, comparable
        // to the jump line. Independent of the jump gap filter above.
        double gx = 0.0, gy = 0.0;
        if (interpolate_gt(stamp, gx, gy)) {
            double ex = x - gx;
            double ey = y - gy;
            double acc = std::sqrt(ex*ex + ey*ey);
            slam_accuracy_ = acc;

            slam_accuracy_history_.append(qMakePair(current_time, acc));
            max_slam_jump_ = std::max(max_slam_jump_, acc);  // shared y-scale

            const double window = 30.0;
            while (!slam_accuracy_history_.isEmpty() && (current_time - slam_accuracy_history_.first().first > window)) {
                slam_accuracy_history_.removeFirst();
            }

            slam_jump_dirty_ = true;
        }

        prev_x_ = x;
        prev_y_ = y;
        prev_yaw_ = yaw;
        prev_vx_ = vx;
        prev_vy_ = vy;
        prev_stamp_ = stamp;
        has_prev_state_ = true;
    }, Qt::QueuedConnection);
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
void StatsInterface::stats_callback(double active_vertices, double active_edges, double observed_landmarks, double unmatched_landmarks, 
    double optimization_time, double data_association_time)
{
    active_vertices_ = active_vertices;
    active_edges_ = active_edges;
    observed_landmarks_ = observed_landmarks;
    unmatched_landmarks_ = unmatched_landmarks;
    optimization_time_ = optimization_time;
    data_association_time_ = data_association_time;

    // Collect the sample now (cheap); defer the repaint to the render timer.
    collect_optimizer_sample();
    telemetry_dirty_ = true;
    optimizer_dirty_ = true;
}

QVector<QPointF> StatsInterface::decimate_series(
    const QVector<QPair<double, double>>& hist, double current_time, double window,
    int w, int h, double vmin, double vrange)
{
    // Reduce the series to at most one vertex per horizontal pixel, keeping the
    // peak of each column so spikes stay visible. Bounds the draw cost to ~w
    // vertices regardless of the input rate.
    QVector<QPointF> pts;
    if (hist.isEmpty() || w <= 0) return pts;
    pts.reserve(w + 2);
    bool have_col = false;
    int cur_col = 0;
    double best_x = 0.0, best_norm = 0.0;
    for (const auto &p : hist) {
        double x = ((p.first - (current_time - window)) / window) * w;
        double norm = (p.second - vmin) / vrange;
        int col = static_cast<int>(x);
        if (!have_col || col != cur_col) {
            if (have_col) pts.append(QPointF(best_x, h - best_norm * h));
            cur_col = col;
            best_x = x;
            best_norm = norm;
            have_col = true;
        } else if (norm > best_norm) {
            best_norm = norm;
            best_x = x;
        }
    }
    if (have_col) pts.append(QPointF(best_x, h - best_norm * h));
    return pts;
}

double StatsInterface::mean_of(const QVector<QPair<double, double>>& hist)
{
    if (hist.isEmpty()) return 0.0;
    double sum = 0.0;
    for (const auto& p : hist) sum += p.second;
    return sum / hist.size();
}

// Linearly interpolate the buffered ground-truth pose to time t. Returns false
// if there is no ground truth close enough in time (buffer empty, or t further
// than kGtMatchTol beyond either end). gt_buffer_ is in timestamp order.
bool StatsInterface::interpolate_gt(double t, double& gx, double& gy)
{
    const int n = gt_buffer_.size();
    if (n == 0) return false;

    const GtSample& first = gt_buffer_.first();
    const GtSample& last  = gt_buffer_.last();

    // Before/after the buffered range: clamp to the nearest end if close enough.
    if (t <= first.stamp) {
        if (first.stamp - t > kGtMatchTol) return false;
        gx = first.x; gy = first.y;
        return true;
    }
    if (t >= last.stamp) {
        if (t - last.stamp > kGtMatchTol) return false;
        gx = last.x; gy = last.y;
        return true;
    }

    // Interpolate between the two samples bracketing t.
    for (int i = 0; i < n - 1; ++i) {
        const GtSample& a = gt_buffer_[i];
        const GtSample& b = gt_buffer_[i + 1];
        if (t >= a.stamp && t <= b.stamp) {
            double span = b.stamp - a.stamp;
            double u = (span > 1e-9) ? (t - a.stamp) / span : 0.0;
            gx = a.x + u * (b.x - a.x);
            gy = a.y + u * (b.y - a.y);
            return true;
        }
    }
    return false;
}

void StatsInterface::rebuild_jump_background(int w, int h)
{
    jump_bg_ = QPixmap(w, h);
    jump_bg_.fill(Qt::white);
    QPainter painter(&jump_bg_);
    painter.setRenderHint(QPainter::Antialiasing);

    const double slam_range = std::max(max_slam_jump_ - min_slam_jump_, 1e-6);
    const int num_rows = 5;

    // Grid
    painter.setPen(QPen(Qt::lightGray, graph_grid_width_, Qt::DashLine));
    for (int i = 1; i < num_rows; ++i) {
        int y = h - qRound((static_cast<double>(i) / num_rows) * h);
        painter.drawLine(0, y, w, y);
    }

    // Numeric scale on the left
    painter.setPen(Qt::black);
    QFont scale_font = painter.font();
    scale_font.setPointSizeF(std::min(h * 0.075, 12.0));
    painter.setFont(scale_font);
    const int label_x = 5;
    for (int i = 0; i <= num_rows; ++i) {
        double value = min_slam_jump_ + (slam_range * (static_cast<double>(i) / num_rows));
        int y = h - qRound((static_cast<double>(i) / num_rows) * h);
        int text_y = (i == num_rows) ? y + 12 : y;
        painter.drawText(label_x, text_y, QString::number(value, 'f', 4));
    }
}

void StatsInterface::update_slam_jump_graph()
{
    if (!slam_jump_label_) return;

    if (slam_jump_value_label_) {
        slam_jump_value_label_->setText("Slam Jump (Precision): " + QString::number(slam_jump_, 'f', 4));
    }
    if (slam_accuracy_label_) {
        slam_accuracy_label_->setText("Accuracy: " + QString::number(slam_accuracy_, 'f', 4));
    }
    if (slam_precision_mean_label_) {
        slam_precision_mean_label_->setText("Precision (media): " + QString::number(mean_of(slam_jump_history_), 'f', 4));
    }
    if (slam_accuracy_mean_label_) {
        slam_accuracy_mean_label_->setText("Accuracy (media): " + QString::number(mean_of(slam_accuracy_history_), 'f', 4));
    }

    int pixmap_width = slam_jump_label_->width();
    int pixmap_height = slam_jump_label_->height();
    if (pixmap_width <= 0 || pixmap_height <= 0) return;

    double current_time = timer_.elapsed() / 1000.0;
    const double window = 30.0;
    double slam_range = std::max(max_slam_jump_ - min_slam_jump_, 1e-6);

    // Rebuild the cached background (grid + scale text) only when the size or the
    // value range changes; otherwise reuse it — this skips per-frame drawText.
    if (jump_bg_.width() != pixmap_width || jump_bg_.height() != pixmap_height ||
        jump_bg_min_ != min_slam_jump_ || jump_bg_max_ != max_slam_jump_) {
        rebuild_jump_background(pixmap_width, pixmap_height);
        jump_bg_min_ = min_slam_jump_;
        jump_bg_max_ = max_slam_jump_;
    }

    QPixmap pixmap = jump_bg_;
    QPainter painter(&pixmap);   // no Antialiasing hint: cheaper line rasterization

    // Both series are decimated to one vertex per horizontal pixel and share the
    // y-scale (both are pose-error magnitudes). Blue = jump (precision, how much
    // the estimate moves beyond the motion model); red = accuracy (error vs the
    // simulator ground truth /arussim/state).
    QVector<QPointF> jump_pts =
        decimate_series(slam_jump_history_, current_time, window, pixmap_width, pixmap_height, min_slam_jump_, slam_range);
    if (!jump_pts.isEmpty()) {
        painter.setPen(QPen(Qt::blue, pen_size_));
        painter.drawPolyline(jump_pts.constData(), jump_pts.size());
    }

    QVector<QPointF> acc_pts =
        decimate_series(slam_accuracy_history_, current_time, window, pixmap_width, pixmap_height, min_slam_jump_, slam_range);
    if (!acc_pts.isEmpty()) {
        painter.setPen(QPen(Qt::red, pen_size_));
        painter.drawPolyline(acc_pts.constData(), acc_pts.size());
    }

    // Current overall max (jump or accuracy) at the top-left
    if (!jump_pts.isEmpty() || !acc_pts.isEmpty()) {
        painter.setPen(Qt::black);
        painter.drawText(5, 12, QString::number(max_slam_jump_, 'f', 4));
    }

    // Legend (top-right), drawn per frame so it stays on top of the data lines.
    {
        QFont legend_font = painter.font();
        legend_font.setPointSizeF(std::min(pixmap_height * 0.06, 10.0));
        painter.setFont(legend_font);
        QFontMetrics fm(legend_font);
        const QString l1 = "Jump";
        const QString l2 = "Accuracy";
        const int pad = 4;
        const int gap = 6;
        int line_len = std::max(12, pixmap_width / 18);
        int text_w = std::max(fm.horizontalAdvance(l1), fm.horizontalAdvance(l2));
        int row_h = fm.height();
        int box_w = pad + line_len + gap + text_w + pad;
        int box_h = pad + 2 * row_h + pad;
        int box_x = pixmap_width - box_w - 2;
        int box_y = 2;
        painter.setBrush(QColor(255, 255, 255, 220));
        painter.setPen(Qt::black);
        painter.drawRect(box_x, box_y, box_w, box_h);

        int y1 = box_y + pad + row_h / 2;
        painter.setPen(QPen(Qt::blue, pen_size_));
        painter.drawLine(box_x + pad, y1, box_x + pad + line_len, y1);
        painter.setPen(Qt::black);
        painter.drawText(box_x + pad + line_len + gap, y1 + fm.ascent() / 2, l1);

        int y2 = box_y + pad + row_h + row_h / 2;
        painter.setPen(QPen(Qt::red, pen_size_));
        painter.drawLine(box_x + pad, y2, box_x + pad + line_len, y2);
        painter.setPen(Qt::black);
        painter.drawText(box_x + pad + line_len + gap, y2 + fm.ascent() / 2, l2);
    }

    slam_jump_label_->setPixmap(pixmap);
}

void StatsInterface::collect_optimizer_sample()
{
    // Data-only step: runs once per /slam/stats message. The actual repaint is
    // done separately in update_optimizer_time_graph() at a capped frame rate.
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

    // Remove points older than the window
    while (!optimization_time_history_.isEmpty() && (current_time - optimization_time_history_.first().first > window)) {
        optimization_time_history_.removeFirst();
    }
}

void StatsInterface::render_tick()
{
    // Coalesce the data collected since the last frame into a single repaint.
    // Only the graphs whose data actually changed are redrawn.
    if (telemetry_dirty_) { update_telemetry_labels(); telemetry_dirty_ = false; }
    if (optimizer_dirty_) { update_optimizer_time_graph(); optimizer_dirty_ = false; }
    if (slam_jump_dirty_) { update_slam_jump_graph(); slam_jump_dirty_ = false; }
}

void StatsInterface::rebuild_optimizer_background(int w, int h)
{
    opt_bg_ = QPixmap(w, h);
    opt_bg_.fill(Qt::white);
    QPainter painter(&opt_bg_);
    painter.setRenderHint(QPainter::Antialiasing);

    int num_rows = qRound(max_optimization_time_ / 0.025);
    if (num_rows <= 0) num_rows = 1;
    double step_y = static_cast<double>(h) / num_rows;

    // Grid lines
    painter.setPen(QPen(Qt::lightGray, graph_grid_width_, Qt::DashLine));
    for (int j = 1; j < num_rows; ++j) {
        int y_line = h - qRound(j * step_y);
        painter.drawLine(0, y_line, w, y_line);
    }

    // Numbers 0.025, 0.050, 0.075... aligned with each grid row
    painter.setPen(Qt::black);
    for (int j = 0; j <= num_rows; ++j) {
        double val = j * 0.025;
        int y_pos = h - (j * step_y);
        int text_y = (j == num_rows) ? y_pos + 12 : y_pos;
        painter.drawText(5, text_y, QString::number(val, 'f', 3));
    }
}

void StatsInterface::update_optimizer_time_graph()
{
    // Render-only step (data is collected in collect_optimizer_sample()).
    double current_time = timer_.elapsed() / 1000.0;
    const double window = 30.0;

    // Configure plot dimensions
    int pixmap_width = slam_graph_label_->width();
    int pixmap_height = slam_graph_label_->height();
    if (pixmap_width <= 0 || pixmap_height <= 0) return;

    // Rebuild the cached background (grid + scale text) only when the size or the
    // range (max_optimization_time_) changes; otherwise reuse it — this skips the
    // per-frame grid drawing and drawText, the dominant cost at high frame rates.
    if (opt_bg_.width() != pixmap_width || opt_bg_.height() != pixmap_height ||
        opt_bg_max_ != max_optimization_time_) {
        rebuild_optimizer_background(pixmap_width, pixmap_height);
        opt_bg_max_ = max_optimization_time_;
    }

    QPixmap pixmap = opt_bg_;
    QPainter painter(&pixmap);   // no Antialiasing hint: cheaper line rasterization

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
    active_vertices_label_->setText("Active Vertices: " + QString::number(active_vertices_, 'f', 0));
    active_edges_label_->setText("Active Edges: " + QString::number(active_edges_, 'f', 0));
    observed_landmarks_label_->setText("Observed Landmarks: " + QString::number(observed_landmarks_, 'f', 0));
    unmatched_landmarks_label_->setText("Unmatched Landmarks: " + QString::number(unmatched_landmarks_, 'f', 0));
    optimization_time_label_->setText("Optimization Time: " + QString::number(optimization_time_, 'f', 5));
    data_association_time_label_->setText("Data Association Time: " + QString::number(data_association_time_, 'f', 5));
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
    slam_jump_history_.clear();
    slam_accuracy_history_.clear();
    if (slam_jump_label_) {
        slam_jump_label_->clear();
    }

    // reset slam jump / accuracy state
    slam_jump_ = 0.0;
    slam_accuracy_ = 0.0;
    gt_buffer_.clear();
    has_prev_state_ = false;
    prev_x_ = 0.0;
    prev_y_ = 0.0;
    prev_yaw_ = 0.0;
    prev_vx_ = 0.0;
    prev_vy_ = 0.0;
    prev_stamp_ = 0.0;
    min_slam_jump_ = 0.0;
    max_slam_jump_ = 0.1;

    slam_jump_dirty_ = false;
    optimizer_dirty_ = false;
    telemetry_dirty_ = false;

    active_vertices_label_->setText("Active Vertices: " + QString::number(0.0, 'f', 0));
    active_edges_label_->setText("Active Edges: " + QString::number(0.0, 'f', 0));
    observed_landmarks_label_->setText("Observed Landmarks: " + QString::number(0.0, 'f', 0));
    unmatched_landmarks_label_->setText("Unmatched Landmarks: " + QString::number(0.0, 'f', 0));
    optimization_time_label_->setText("Optimization Time: " + QString::number(0.0, 'f', 5));
    data_association_time_label_->setText("Data Association Time: " + QString::number(0.0, 'f', 5));
    if (slam_jump_value_label_) {
        slam_jump_value_label_->setText("Slam Jump (Precision): " + QString::number(0.0, 'f', 4));
    }
    if (slam_accuracy_label_) {
        slam_accuracy_label_->setText("Accuracy: " + QString::number(0.0, 'f', 4));
    }
    if (slam_precision_mean_label_) {
        slam_precision_mean_label_->setText("Precision (media): " + QString::number(0.0, 'f', 4));
    }
    if (slam_accuracy_mean_label_) {
        slam_accuracy_mean_label_->setText("Accuracy (media): " + QString::number(0.0, 'f', 4));
    }
    
}

}  // namespace stats_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(stats_interface::StatsInterface, rviz_common::Panel)