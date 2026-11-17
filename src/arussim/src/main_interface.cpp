#include <arussim/main_interface.hpp>
namespace main_interface
{

MainInterface::MainInterface(QWidget* parent) : Panel(parent)
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

    // Virtual terminal output
    process_output_text_edit_ = new QTextEdit(this);
    process_output_text_edit_->setReadOnly(true);
    process_output_text_edit_->setMinimumHeight(100);
    main_grid->addWidget(process_output_text_edit_, 1, 0);

    // Buttons
    auto button_grid = new QGridLayout();
    button_grid->setContentsMargins(grid_margin_, grid_margin_, grid_margin_, grid_margin_);
    button_grid->setSpacing(grid_margin_);
    button_grid->setAlignment(Qt::AlignTop);

    // Launch button
    launch_button_ = new QPushButton("Launch Simulation", this);
    launch_button_->setFixedHeight(rviz_height_ * 0.025);
    main_layout->addWidget(launch_button_);
    connect(launch_button_, &QPushButton::clicked, this, &MainInterface::launch_button_clicked);
    button_grid->addWidget(launch_button_, 1, 0);

    // Stop button
    stop_button_ = new QPushButton("Stop Simulation", this);
    stop_button_->setFixedHeight(rviz_height_ * 0.025);
    main_layout->addWidget(stop_button_);
    connect(stop_button_, &QPushButton::clicked, this, &MainInterface::stop_button_clicked);
    button_grid->addWidget(stop_button_, 2, 0);

    // Reset button
    reset_button_ = new QPushButton("Reset Simulation", this);
    reset_button_->setFixedHeight(rviz_height_ * 0.025);
    main_layout->addWidget(reset_button_);
    connect(reset_button_, &QPushButton::clicked, this, &MainInterface::reset_button_clicked);
    button_grid->addWidget(reset_button_, 2, 1);

    // Circuit selection 
    circuit_select_ = new QComboBox(this);
    circuit_select_->setFixedHeight(rviz_height_ * 0.025);
    circuit_select_->setPlaceholderText("Choose a circuit");

    std::string pkg_arussim = ament_index_cpp::get_package_share_directory("arussim");
    QString tracks_root = QString::fromStdString(pkg_arussim + "/resources/tracks/");

    auto * mdl = new QStandardItemModel(circuit_select_);
    circuit_select_->setModel(mdl);

    auto add_header = [&](const QString & label) {
        auto * it = new QStandardItem("── " + label + " ──");
        it->setEnabled(false);
        it->setForeground(QBrush(QColor(110, 90, 50)));
        QFont f = it->font(); f.setItalic(true); f.setBold(true); it->setFont(f);
        it->setBackground(QBrush(QColor(225, 218, 202)));
        mdl->appendRow(it);
    };

    auto list_tracks = [&](const QString & subdir) {
        return QDir(tracks_root + subdir).entryList(
            QStringList() << "*.pcd", QDir::Files, QDir::Name);
    };

    auto add_track = [&](const QString & subdir, const QString & file) {
        auto * it = new QStandardItem(file);
        it->setData(subdir + "/" + file, Qt::UserRole);
        mdl->appendRow(it);
        return it;
    };

    // Recorded tracks are always visible
    QStringList recorded_files = list_tracks("recorded");
    if (!recorded_files.isEmpty()) {
        add_header("Recorded (Real Tracks)");
        for (const QString & f : recorded_files) add_track("recorded", f);
    }

    // Generated tracks stay collapsed until the toggle row is clicked
    QStringList generated_files = list_tracks("generated");
    if (!generated_files.isEmpty()) {
        generated_toggle_ = new QStandardItem();
        generated_toggle_->setForeground(QBrush(QColor(110, 90, 50)));
        QFont f = generated_toggle_->font(); f.setBold(true); generated_toggle_->setFont(f);
        generated_toggle_->setBackground(QBrush(QColor(225, 218, 202)));
        mdl->appendRow(generated_toggle_);
        for (const QString & g : generated_files) generated_items_.append(add_track("generated", g));
    }

    auto refresh_generated = [this]() {
        if (!generated_toggle_) return;
        generated_toggle_->setText(QString("%1 Generated (%2)")
            .arg(generated_expanded_ ? "\u25BE" : "\u25B8")
            .arg(generated_items_.size()));
        auto * view = qobject_cast<QListView*>(circuit_select_->view());
        if (!view) return;
        for (auto * it : generated_items_) view->setRowHidden(it->row(), !generated_expanded_);
    };
    refresh_generated();

    connect(circuit_select_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this, mdl, refresh_generated](int idx) {
                // Toggle row: expand/collapse generated tracks and keep the popup open
                if (generated_toggle_ && idx >= 0 && mdl->item(idx) == generated_toggle_) {
                    generated_expanded_ = !generated_expanded_;
                    refresh_generated();
                    circuit_select_->blockSignals(true);
                    circuit_select_->setCurrentIndex(last_circuit_index_);
                    circuit_select_->blockSignals(false);
                    QTimer::singleShot(0, circuit_select_, [this]() { circuit_select_->showPopup(); });
                    return;
                }
                QString val = circuit_select_->itemData(idx, Qt::UserRole).toString();
                if (val.isEmpty()) return;
                last_circuit_index_ = idx;
                auto msg = std_msgs::msg::String();
                msg.data = val.toStdString();
                circuit_pub_->publish(msg);
                auto rst = std_msgs::msg::Bool(); rst.data = true;
                reset_pub_->publish(rst);
                RCLCPP_INFO(rclcpp::get_logger("MainInterface"),
                    "%sCircuit: %s%s", cyan.c_str(), val.toStdString().c_str(), reset.c_str());
          });

    button_grid->addWidget(circuit_select_, 1, 1);

    // Launch selection
    launch_select_ = new QComboBox(this);
    launch_select_->setFixedHeight(rviz_height_ * 0.025);
    std::string package_path_launch = ament_index_cpp::get_package_share_directory("common_meta");
    QString launch_path = QString::fromStdString(package_path_launch + "/launch/");
    QDir launch_dir(launch_path);
    QStringList launch_files = launch_dir.entryList(QStringList() << "*sim*.py", QDir::Files);
    launch_select_->addItems(launch_files);
    launch_select_->setCurrentText("simulation_launch.py");
    button_grid->addWidget(launch_select_, 0, 0, 1, 2);

    main_grid->addLayout(button_grid, 2, 0);

    main_layout->addLayout(main_grid);
}

MainInterface::~MainInterface() = default;

void MainInterface::onInitialize()
{
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

    // Publishers
    launch_pub_ = node->create_publisher<std_msgs::msg::Bool>("/arussim/launch", 1);
    reset_pub_ = node->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);
    circuit_pub_ = node->create_publisher<std_msgs::msg::String>("/arussim/circuit", 1);

    // Subscribers
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
void MainInterface::update_lap_time_labels(double lap_time_)
{
    lap_counter_++;
    lap_label_->setText("Lap: " + QString::number(lap_counter_));
    last_lap_time_ = lap_time_;
    last_lt_label_->setText("Last Lap Time: " + QString::number(last_lap_time_, 'f', 3) + "s");

    static double skidpad_lap_2 = 0.0;
    const bool is_skidpad = circuit_select_->currentText().startsWith("skidpad", Qt::CaseInsensitive) || launch_select_->currentText().contains("skidpad", Qt::CaseInsensitive);
    const bool valid_lap = !is_skidpad || lap_counter_ == 4;
    if (is_skidpad && lap_counter_ == 2) skidpad_lap_2 = lap_time_;
    const double best_candidate = is_skidpad ? (skidpad_lap_2 + lap_time_) / 2.0 : lap_time_;

    if (valid_lap && best_candidate < best_lap_time_) {
        best_lap_time_ = best_candidate;
        best_lt_label_->setText("Best Lap Time: " + QString::number(best_lap_time_, 'f', 3) + "s");
        last_lt_label_->setStyleSheet("color: purple;");
    } else {
        last_lt_label_->setStyleSheet("color: black;");
    }
    last_lt_label_->setText("Last Lap Time: " + QString::number(lap_time_, 'f', 3) + "s");
}


/**
 * @brief Callback for the launch button
 * 
 */
void MainInterface::launch_button_clicked()
{
    
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    launch_pub_->publish(msg);

    if (simulation_process_ == nullptr) {
        simulation_process_ = new QProcess(this);
        // Merge standard output and error
        simulation_process_->setProcessChannelMode(QProcess::MergedChannels);
        // Connect signal to capture output
        connect(simulation_process_, &QProcess::readyReadStandardOutput, this, &MainInterface::process_output);
        
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
void MainInterface::stop_button_clicked()
{
    if (simulation_process_) {
        kill(static_cast<pid_t>(simulation_process_->processId()), SIGINT);
        simulation_process_->waitForFinished();
        simulation_process_->deleteLater();
        simulation_process_ = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("MainInterface"), "%sSimulation stopped%s", red.c_str(), reset.c_str());
    }
}

/**
 * @brief Callback for the reset button
 * 
 */
void MainInterface::reset_button_clicked()
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

    RCLCPP_INFO(rclcpp::get_logger("MainInterface"), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

/**
 * @brief Process the output from the simulation
 * 
 */
void MainInterface::process_output()
{
    if(simulation_process_) {
        QByteArray output = simulation_process_->readAllStandardOutput();
        QString text = QString::fromLocal8Bit(output);
        // Remove ANSI sequences
        QRegularExpression ansiRegex("\x1B\\[[0-9;]*m");
        text.remove(ansiRegex);
        process_output_text_edit_->append(text);
    }
}

}  // namespace main_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(main_interface::MainInterface, rviz_common::Panel)