/**
 * @file extended_interface.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Extended interface with some controls and car data.
 * @version 0.1
 * @date 2024-11-11
 */
#include "arussim/extended_interface.hpp"


/**
 * @brief Construct a new Extended Interface:: Extended Interface object
 * 
 * @param parent 
 */
ExtendedInterface::ExtendedInterface(QWidget* parent) : QWidget(parent), Node("extended_interface") {
    // Set window size
    setFixedWidth(window_width_);
    setFixedHeight(window_height_);

    // Set font
    QFont custom_font_("Montserrat Regular", 13);
    QFont bold_font_ = custom_font_;
    bold_font_.setBold(true);

    // Set icon
    setWindowIcon(QIcon("ARUSSim/src/track_editor/icons/icon.png"));

    //Time per lap labels
    last_lt_label_ = new QLabel("Last Lap Time: 0s", this);
    last_lt_label_->setFont(bold_font_);
    last_lt_label_->setFixedSize(window_width_ * 0.55, margins_);
    last_lt_label_->move(margins_, margins_);

    lap_label_ = new QLabel("Lap: 0", this);
    lap_label_->setFont(bold_font_);
    lap_label_->setFixedSize(window_width_ * 0.55, margins_);
    lap_label_->move(window_width_ * 0.55, margins_);

    best_lt_label_ = new QLabel("Best Lap Time: 0s", this);
    best_lt_label_->setFont(bold_font_);
    best_lt_label_->setFixedSize(window_width_ * 0.55, margins_);
    best_lt_label_->move(margins_, margins_*2);

    hit_cones_label_ = new QLabel("Hit cones: 0", this);
    hit_cones_label_->setFont(bold_font_);
    hit_cones_label_->setFixedSize(window_width_ * 0.55, margins_);
    hit_cones_label_->move(window_width_ * 0.55, margins_*2);

    // Telemetry bars
    telemetry_label_ = new QLabel("Telemetry", this);
    telemetry_label_->setFont(custom_font_);
    telemetry_label_->move(margins_ * 0.95, margins_*4);

    telemetry_container_fl_ = new QWidget(this);
    telemetry_container_fl_->setFixedSize(window_width_ * 0.425, window_height_ * 0.15);
    telemetry_container_fl_->setStyleSheet("background-color: lightgray;");
    telemetry_container_fl_->move(margins_, margins_ * 5);
    telemetry_bar_fl_ = new QWidget(telemetry_container_fl_);
    telemetry_bar_fl_->setFixedWidth(window_width_ * 0.425);
    telemetry_bar_fl_->move(0, center_y_);

    telemetry_container_fr_ = new QWidget(this);
    telemetry_container_fr_->setFixedSize(window_width_ * 0.425, window_height_ * 0.15);
    telemetry_container_fr_->setStyleSheet("background-color: lightgray;");
    telemetry_container_fr_->move(window_width_ * 0.525, margins_ * 5);
    telemetry_bar_fr_ = new QWidget(telemetry_container_fr_);
    telemetry_bar_fr_->setFixedWidth(window_width_ * 0.425);
    telemetry_bar_fr_->move(0, center_y_);

    telemetry_rear_container_position_y_ = window_height_ * 0.15 + margins_ * 6;

    telemetry_container_rl_ = new QWidget(this);
    telemetry_container_rl_->setFixedSize(window_width_ * 0.425, window_height_ * 0.15);
    telemetry_container_rl_->setStyleSheet("background-color: lightgray;");
    telemetry_container_rl_->move(margins_, telemetry_rear_container_position_y_);
    telemetry_bar_rl_ = new QWidget(telemetry_container_rl_);
    telemetry_bar_rl_->setFixedWidth(window_width_ * 0.425);
    telemetry_bar_rl_->move(0, center_y_);

    telemetry_container_rr_ = new QWidget(this);
    telemetry_container_rr_->setFixedSize(window_width_ * 0.425, window_height_ * 0.15);
    telemetry_container_rr_->setStyleSheet("background-color: lightgray;");
    telemetry_container_rr_->move(window_width_ * 0.525, telemetry_rear_container_position_y_);
    telemetry_bar_rr_ = new QWidget(telemetry_container_rr_);
    telemetry_bar_rr_->setFixedWidth(window_width_ * 0.425);
    telemetry_bar_rr_->move(0, center_y_);

    // Telemetry labels
    telemetry_parameters_position_y_ = telemetry_rear_container_position_y_ + window_height_ * 0.15 + margins_;

    vx_label_ = new QLabel("vx: 0 m/s", this);
    vx_label_->setFont(custom_font_);
    vx_label_->setFixedSize(window_width_ * 0.425, margins_);
    vx_label_->move(margins_, telemetry_parameters_position_y_);

    vy_label_ = new QLabel("vy: 0 m/s", this);
    vy_label_->setFont(custom_font_);
    vy_label_->setFixedSize(window_width_ * 0.425, margins_);
    vy_label_->move(window_width_ * 0.525, telemetry_parameters_position_y_);
    
    ax_label_ = new QLabel("ax: 0 m/s^2", this);
    ax_label_->setFont(custom_font_);
    ax_label_->setFixedSize(window_width_ * 0.425, margins_);
    ax_label_->move(margins_, telemetry_parameters_position_y_ + margins_);

    ay_label_ = new QLabel("ay: 0 m/s^2", this);
    ay_label_->setFont(custom_font_);
    ay_label_->setFixedSize(window_width_ * 0.425, margins_);
    ay_label_->move(window_width_ * 0.525, telemetry_parameters_position_y_ + margins_);

    r_label_ = new QLabel("r: 0 rad/s", this);
    r_label_->setFont(custom_font_);
    r_label_->setFixedSize(window_width_ * 0.425, margins_);
    r_label_->move(margins_, telemetry_parameters_position_y_ + margins_ * 2);
    
    delta_label_ = new QLabel("delta: 0º", this);
    delta_label_->setFont(custom_font_);
    delta_label_->setFixedSize(window_width_ * 0.425, margins_);
    delta_label_->move(window_width_ * 0.525, telemetry_parameters_position_y_ + margins_ * 2);

    // Launch button
    launch_button_position_y_ = telemetry_parameters_position_y_ + margins_ * 4;

    reset_button_ = new QPushButton("Launch Simulation", this);
    reset_button_->move(margins_, launch_button_position_y_);
    reset_button_->setFixedSize(window_width_ * 0.9, window_height_ * 0.05);
    reset_button_->setFont(custom_font_);
    connect(reset_button_, &QPushButton::clicked, this, &ExtendedInterface::launch_button_clicked);


    // timer slider
    timer_setter_position_y_ = launch_button_position_y_ + window_height_ * 0.05 + margins_;

    timer_label_ = new QLabel("Automatic Simulation: x" + QString::number(simulation_speed_multiplier, 'f', 1), this);
    timer_label_->setFont(custom_font_);
    timer_label_->setFixedSize(window_width_*0.9, margins_);
    timer_label_->move(margins_, timer_setter_position_y_);

    timer_setter_ = new QSlider(Qt::Horizontal, this);
    timer_setter_->setRange(0, 100);
    timer_setter_->setValue(static_cast<int>(simulation_speed_multiplier * 10));
    timer_setter_->setGeometry(margins_, timer_setter_position_y_ + margins_, window_width_ * 0.9, margins_);
    timer_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(timer_setter_, &QSlider::valueChanged, this, &ExtendedInterface::timer_set);

    // Slider 1
    a_label_ = new QLabel("a: 0", this);
    a_label_->setFont(custom_font_);
    a_label_->setFixedSize(window_width_*0.9, margins_);
    a_label_->move(margins_, timer_setter_position_y_ + margins_ * 3);

    a_setter_ = new QSlider(Qt::Horizontal, this);
    a_setter_->setRange(0, 100);
    a_setter_->setValue(static_cast<int>(kA));
    a_setter_->setGeometry(margins_, timer_setter_position_y_ + margins_*4, window_width_ * 0.9, margins_);
    a_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(a_setter_, &QSlider::valueChanged, this, &ExtendedInterface::a_set);

    // Slider 2
    b_label_ = new QLabel("b: 0", this);
    b_label_->setFont(custom_font_);
    b_label_->setFixedSize(window_width_*0.9, margins_);
    b_label_->move(margins_, timer_setter_position_y_ + margins_ * 6);

    b_setter_ = new QSlider(Qt::Horizontal, this);
    b_setter_->setRange(0, 100);
    b_setter_->setValue(static_cast<int>(kB));
    b_setter_->setGeometry(margins_, timer_setter_position_y_ + margins_*7, window_width_ * 0.9, margins_);
    b_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(b_setter_, &QSlider::valueChanged, this, &ExtendedInterface::b_set);

    //Stop button
    ab_button_position_y_ = timer_setter_position_y_ + margins_ * 9;

    a_button_ = new QPushButton("Stop Simulation", this);
    a_button_->move(margins_, ab_button_position_y_);
    a_button_->setFixedSize(window_width_ * 0.425, window_height_ * 0.05);
    a_button_->setFont(custom_font_);
    connect(a_button_, &QPushButton::clicked, this, &ExtendedInterface::stop_button_clicked);

    //Reset button
    b_button_ = new QPushButton("Reset", this);
    b_button_->move(window_width_ * 0.525, ab_button_position_y_);
    b_button_->setFixedSize(window_width_ * 0.425, window_height_ * 0.05);
    b_button_->setFont(custom_font_);
    connect(b_button_, &QPushButton::clicked, this, &ExtendedInterface::reset_button_clicked);




    // Publisher
    reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arussim/reset", 1);

    // Subscriber
    cmd_sub_ = this->create_subscription<arussim_msgs::msg::Cmd>(
        "/arussim/cmd", 1, 
        [this](const arussim_msgs::msg::Cmd::SharedPtr msg) { 
            QMetaObject::invokeMethod(this, [this, msg]() {
                update_telemetry_bar(msg->acc, msg->acc, msg->acc, msg->acc);
            }, Qt::QueuedConnection);
        }
    );

    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        [this](const arussim_msgs::msg::State::SharedPtr msg) { 
            QMetaObject::invokeMethod(this, [this, msg]() {
                update_telemetry_labels(msg->vx, msg->vy, msg->r, msg->ax, msg->ay, msg->delta);
            }, Qt::QueuedConnection);
        }
    );

    lap_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/lap_time", 1, 
        [this](const std_msgs::msg::Float32::SharedPtr msg) { 
            QMetaObject::invokeMethod(this, [this, msg]() {
                update_lap_time_labels(msg->data);
            }, Qt::QueuedConnection);
        }
    );

    hit_cones_bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
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


    // Client
    timer_client_ = this->create_client<arussim_msgs::srv::SetTimer>("arussim/set_timer");

    // Activate window
    QTimer::singleShot(3000, [this]() {
        raise();
        activateWindow();
    });

    // Move the window to the right side of the screen
    move(window_position_x_, 0);
}

/**
 * @brief setter for the timer parameter
 * 
 * @param value_ 
 */
void ExtendedInterface::timer_set(int value) {
    double timer_value = value / 10.0;
    timer_label_->setText("Automatic Simulation: x" + QString::number(timer_value, 'f', 1));

    auto request = std::make_shared<arussim_msgs::srv::SetTimer::Request>();
    request->timer = timer_value;

    auto future = timer_client_->async_send_request(
        request,
        [this](rclcpp::Client<arussim_msgs::srv::SetTimer>::SharedFuture future) {
            auto result = future.get();
            if (!result->success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set timer: %s", result->message.c_str());
            }
        }
    );
}

/**
 * @brief setter for the a parameter
 * 
 * @param value_ 
 */
void ExtendedInterface::a_set(int value_) {
    kA = static_cast<double>(value_);
    a_label_->setText("a: " + QString::number(value_));
}

/**
 * @brief setter for the b parameter
 * 
 * @param value_ 
 */
void ExtendedInterface::b_set(int value_) {
    kB = static_cast<double>(value_);
    b_label_->setText("b: " + QString::number(value_));
}

/**
 * @brief Update the telemetry bars
 * 
 * @param fl_param_ 
 * @param fr_param_ 
 * @param rl_param_ 
 * @param rr_param_ 
 */
void ExtendedInterface::update_telemetry_bar(double fl_param_, double fr_param_, double rl_param_, double rr_param_)
{
    fr_param_ = fl_param_;
    rl_param_ = fl_param_;
    rr_param_ = fl_param_;

    // Front Left
    double height_fl = std::abs(fl_param_) * scale_factor_;
    height_fl = std::min(height_fl, max_bar_height_);
    telemetry_bar_fl_->setFixedHeight(static_cast<int>(height_fl));
    if (fl_param_ >= 0) {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), center_y_ - height_fl);
        telemetry_bar_fl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), center_y_);
        telemetry_bar_fl_->setStyleSheet("background-color: red;");
    }

    // Front Right
    double height_fr = std::abs(fr_param_) * scale_factor_;
    height_fr = std::min(height_fr, max_bar_height_);
    telemetry_bar_fr_->setFixedHeight(static_cast<int>(height_fr));
    if (fr_param_ >= 0) {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), center_y_ - height_fr);
        telemetry_bar_fr_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), center_y_);
        telemetry_bar_fr_->setStyleSheet("background-color: red;");
    }

    // Rear Left
    double height_rl = std::abs(rl_param_) * scale_factor_;
    height_rl = std::min(height_rl, max_bar_height_);
    telemetry_bar_rl_->setFixedHeight(static_cast<int>(height_rl));
    if (rl_param_ >= 0) {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), center_y_ - height_rl);
        telemetry_bar_rl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), center_y_);
        telemetry_bar_rl_->setStyleSheet("background-color: red;");
    }

    // Rear Right
    double height_rr = std::abs(rr_param_) * scale_factor_;
    height_rr = std::min(height_rr, max_bar_height_);
    telemetry_bar_rr_->setFixedHeight(static_cast<int>(height_rr));
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
void ExtendedInterface::update_telemetry_labels(double vx_, double vy_, double r_, double ax_, double ay_, double delta_)
{
    vx_label_->setText("vx: " + QString::number(vx_, 'f', 3) + " m/s");
    vy_label_->setText("vy: " + QString::number(vy_, 'f', 3) + " m/s");
    ax_label_->setText("ax: " + QString::number(ax_, 'f', 3) + " m/s^2");
    ay_label_->setText("ay: " + QString::number(ay_, 'f', 3) + " m/s^2");
    r_label_->setText("r: " + QString::number(r_, 'f', 3) + " rad/s");
    delta_label_->setText("delta: " + QString::number(delta_ * 180.0 / M_PI, 'f', 3) + "º");
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

    RCLCPP_INFO(this->get_logger(), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

/**
 * @brief Callback for the a button
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
 * @brief Callback for the b button
 * 
 */
void ExtendedInterface::stop_button_clicked()
{
    if (simulation_process_) {
        kill(static_cast<pid_t>(simulation_process_->processId()), SIGINT);
        simulation_process_->waitForFinished();
        simulation_process_->deleteLater();
        simulation_process_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "%sSimulation stopped%s", red.c_str(), reset.c_str());
    }
}

/**
 * @brief Main function
 * 
 * This initializes the ROS 2 system and starts spinning the Extended Interface node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application. 
 */
int main(int argc, char * argv[])
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExtendedInterface>();
    node->show();

    QTimer rosShutdownTimer;
    QObject::connect(&rosShutdownTimer, &QTimer::timeout, [&]() {
        if (!rclcpp::ok()) {
            app.quit();
        }
    });
    rosShutdownTimer.start(100);

    std::thread rclcpp_thread([&]() {
        rclcpp::spin(node);
        rclcpp::shutdown();
        app.quit();
    });

    int result = app.exec();
    rclcpp_thread.join();

    return result;
}
