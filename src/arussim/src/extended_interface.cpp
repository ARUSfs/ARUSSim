#include "arussim/extended_interface.hpp"

ExtendedInterface::ExtendedInterface(QWidget* parent) : QWidget(parent), Node("extended_interface") {
    // Set window size
    QScreen* screen = QGuiApplication::primaryScreen();
    setFixedWidth(400);
    setFixedHeight(screen->availableGeometry().height());

    // Set font
    QFont customFont("Montserrat Regular", 13);

    // Reset button
    reset_button_ = new QPushButton("Reset", this);
    reset_button_->move(margins_, 500);
    reset_button_->setFixedSize(300, 40);
    reset_button_->setFont(customFont);
    connect(reset_button_, &QPushButton::clicked, this, &ExtendedInterface::reset_button_clicked);

    // Telemetry bars
    telemetry_label_ = new QLabel("Telemetry", this);
    telemetry_label_->setFont(customFont);
    telemetry_label_->move(margins_ - 15, 40);

    telemetry_container_fl_ = new QWidget(this);
    telemetry_container_fl_->setFixedSize(containerWidth_, containerHeight_);
    telemetry_container_fl_->setStyleSheet("background-color: lightgray;");
    telemetry_container_fl_->move(margins_, 75);
    telemetry_bar_fl_ = new QWidget(telemetry_container_fl_);
    telemetry_bar_fl_->setFixedWidth(containerWidth_);
    telemetry_bar_fl_->move(0, centerY_);

    telemetry_container_fr_ = new QWidget(this);
    telemetry_container_fr_->setFixedSize(containerWidth_, containerHeight_);
    telemetry_container_fr_->setStyleSheet("background-color: lightgray;");
    telemetry_container_fr_->move(225, 75);
    telemetry_bar_fr_ = new QWidget(telemetry_container_fr_);
    telemetry_bar_fr_->setFixedWidth(containerWidth_);
    telemetry_bar_fr_->move(0, centerY_);

    telemetry_container_rl_ = new QWidget(this);
    telemetry_container_rl_->setFixedSize(containerWidth_, containerHeight_);
    telemetry_container_rl_->setStyleSheet("background-color: lightgray;");
    telemetry_container_rl_->move(margins_, 250);
    telemetry_bar_rl_ = new QWidget(telemetry_container_rl_);
    telemetry_bar_rl_->setFixedWidth(containerWidth_);
    telemetry_bar_rl_->move(0, centerY_);

    telemetry_container_rr_ = new QWidget(this);
    telemetry_container_rr_->setFixedSize(containerWidth_, containerHeight_);
    telemetry_container_rr_->setStyleSheet("background-color: lightgray;");
    telemetry_container_rr_->move(225, 250);
    telemetry_bar_rr_ = new QWidget(telemetry_container_rr_);
    telemetry_bar_rr_->setFixedWidth(containerWidth_);
    telemetry_bar_rr_->move(0, centerY_);

    // Telemetry labels
    vx_label_ = new QLabel("vx: 0 m/s", this);
    vx_label_->setFont(customFont);
    vx_label_->setFixedSize(150, 25);
    vx_label_->move(margins_, 400);

    vy_label_ = new QLabel("vy: 0 m/s", this);
    vy_label_->setFont(customFont);
    vy_label_->setFixedSize(150, 25);
    vy_label_->move(margins_ + 175, 400);
    
    ax_label_ = new QLabel("ax: 0 m/s^2", this);
    ax_label_->setFont(customFont);
    ax_label_->setFixedSize(150, 25);
    ax_label_->move(margins_, 425);

    ay_label_ = new QLabel("ay: 0 m/s^2", this);
    ay_label_->setFont(customFont);
    ay_label_->setFixedSize(150, 25);
    ay_label_->move(margins_ + 175, 425);

    r_label_ = new QLabel("r: 0 rad/s", this);
    r_label_->setFont(customFont);
    r_label_->setFixedSize(150, 25);
    r_label_->move(margins_, 450);
    
    delta_label_ = new QLabel("delta: 0º", this);
    delta_label_->setFont(customFont);
    delta_label_->setFixedSize(150, 25);
    delta_label_->move(margins_ + 175, 450);


    // FOV slider
    fov_label_ = new QLabel("FOV: " + QString::number(kFOV), this);
    fov_label_->setFont(customFont);
    fov_label_->move(margins_, 575);

    fov_setter_ = new QSlider(Qt::Horizontal, this);
    fov_setter_->setRange(0, 100);
    fov_setter_->setValue(static_cast<int>(kFOV));
    fov_setter_->setGeometry(margins_, 590, 300, 40);
    fov_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(fov_setter_, &QSlider::valueChanged, this, &ExtendedInterface::fov_set);

    // Slider 1
    a_label_ = new QLabel("a: 0", this);
    a_label_->setFont(customFont);
    a_label_->setFixedSize(150, 25);
    a_label_->move(margins_, 625);

    a_setter_ = new QSlider(Qt::Horizontal, this);
    a_setter_->setRange(0, 100);
    a_setter_->setValue(static_cast<int>(kA));
    a_setter_->setGeometry(margins_, 640, 300, 40);
    a_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(a_setter_, &QSlider::valueChanged, this, &ExtendedInterface::a_set);

    // Slider 2
    b_label_ = new QLabel("b: 0", this);
    b_label_->setFont(customFont);
    b_label_->setFixedSize(150, 25);
    b_label_->move(margins_, 675);

    b_setter_ = new QSlider(Qt::Horizontal, this);
    b_setter_->setRange(0, 100);
    b_setter_->setValue(static_cast<int>(kB));
    b_setter_->setGeometry(margins_, 690, 300, 40);
    b_setter_->setStyleSheet("QSlider::handle { background: blue; }");
    connect(b_setter_, &QSlider::valueChanged, this, &ExtendedInterface::b_set);

    //Button 1
    a_button_ = new QPushButton("a", this);
    a_button_->move(margins_, 750);
    a_button_->setFixedSize(150, 40);
    a_button_->setFont(customFont);
    connect(a_button_, &QPushButton::clicked, this, &ExtendedInterface::a_button_clicked);

    //Button 2
    b_button_ = new QPushButton("b", this);
    b_button_->move(margins_ + 175, 750);
    b_button_->setFixedSize(150, 40);
    b_button_->setFont(customFont);
    connect(b_button_, &QPushButton::clicked, this, &ExtendedInterface::b_button_clicked);




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

    // Client
    fov_client_ = this->create_client<arussim_msgs::srv::SetFOV>("arussim/set_fov");

    QTimer::singleShot(1000, [this]() {
        raise();
        activateWindow();
    });
}

void ExtendedInterface::fov_set(int value_) {
    kFOV = static_cast<double>(value_);
    fov_label_->setText("FOV: " + QString::number(value_));

    auto request = std::make_shared<arussim_msgs::srv::SetFOV::Request>();
    request->fov = kFOV;

    auto future = fov_client_->async_send_request(
        request,
        [this](rclcpp::Client<arussim_msgs::srv::SetFOV>::SharedFuture future) {
            auto result = future.get();
            if (!result->success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set FOV: %s", result->message.c_str());
            }
        }
    );
}

void ExtendedInterface::a_set(int value_) {
    kA = static_cast<double>(value_);
    a_label_->setText("a: " + QString::number(value_));
}

void ExtendedInterface::b_set(int value_) {
    kB = static_cast<double>(value_);
    b_label_->setText("b: " + QString::number(value_));
}

void ExtendedInterface::update_telemetry_bar(double fl_param_, double fr_param_, double rl_param_, double rr_param_)
{
    fr_param_ = fl_param_;
    rl_param_ = fl_param_;
    rr_param_ = fl_param_;

    // Front Left
    double height_fl = std::abs(fl_param_) * scaleFactor_;
    height_fl = std::min(height_fl, maxBarHeight_);
    telemetry_bar_fl_->setFixedHeight(static_cast<int>(height_fl));
    if (fl_param_ >= 0) {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), centerY_ - height_fl);
        telemetry_bar_fl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fl_->move(telemetry_bar_fl_->x(), centerY_);
        telemetry_bar_fl_->setStyleSheet("background-color: red;");
    }

    // Front Right
    double height_fr = std::abs(fr_param_) * scaleFactor_;
    height_fr = std::min(height_fr, maxBarHeight_);
    telemetry_bar_fr_->setFixedHeight(static_cast<int>(height_fr));
    if (fr_param_ >= 0) {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), centerY_ - height_fr);
        telemetry_bar_fr_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_fr_->move(telemetry_bar_fr_->x(), centerY_);
        telemetry_bar_fr_->setStyleSheet("background-color: red;");
    }

    // Rear Left
    double height_rl = std::abs(rl_param_) * scaleFactor_;
    height_rl = std::min(height_rl, maxBarHeight_);
    telemetry_bar_rl_->setFixedHeight(static_cast<int>(height_rl));
    if (rl_param_ >= 0) {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), centerY_ - height_rl);
        telemetry_bar_rl_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_rl_->move(telemetry_bar_rl_->x(), centerY_);
        telemetry_bar_rl_->setStyleSheet("background-color: red;");
    }

    // Rear Right
    double height_rr = std::abs(rr_param_) * scaleFactor_;
    height_rr = std::min(height_rr, maxBarHeight_);
    telemetry_bar_rr_->setFixedHeight(static_cast<int>(height_rr));
    if (rr_param_ >= 0) {
        telemetry_bar_rr_->move(telemetry_bar_rr_->x(), centerY_ - height_rr);
        telemetry_bar_rr_->setStyleSheet("background-color: green;");
    } else {
        telemetry_bar_rr_->move(telemetry_bar_rr_->x(), centerY_);
        telemetry_bar_rr_->setStyleSheet("background-color: red;");
    }
}

void ExtendedInterface::update_telemetry_labels(double vx_, double vy_, double r_, double ax_, double ay_, double delta_)
{
    vx_label_->setText("vx: " + QString::number(vx_, 'f', 3) + " m/s");
    vy_label_->setText("vy: " + QString::number(vy_, 'f', 3) + " m/s");
    ax_label_->setText("ax: " + QString::number(ax_, 'f', 3) + " m/s^2");
    ay_label_->setText("ay: " + QString::number(ay_, 'f', 3) + " m/s^2");
    r_label_->setText("r: " + QString::number(r_, 'f', 3) + " rad/s");
    delta_label_->setText("delta: " + QString::number(delta_ * 180.0 / M_PI, 'f', 3) + "º");
}

void ExtendedInterface::reset_button_clicked()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    reset_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%sReset Simulation%s", cyan.c_str(), reset.c_str());
}

void ExtendedInterface::a_button_clicked()
{
    RCLCPP_INFO(this->get_logger(), "%sBotón peruano. Todavía por desperuanizar%s", red.c_str(), reset.c_str());
}

void ExtendedInterface::b_button_clicked()
{
    RCLCPP_INFO(this->get_logger(), "%sBotón peruano. Todavía por desperuanizar%s", red.c_str(), reset.c_str());
}

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
