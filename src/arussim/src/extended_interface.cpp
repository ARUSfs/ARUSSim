#include <arussim/extended_interface.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace rviz_panel_tutorial
{

ExtendedInterface::ExtendedInterface(QWidget* parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  // Create a button and a label for the button
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  // Add those elements to the GUI layout
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the buttonActivated callback being called.
  QObject::connect(button_, &QPushButton::released, this, &ExtendedInterface::buttonActivated);
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

  // Create a String publisher for the output
  publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);

  // Create a String subscription and bind it to the topicCallback inside this class.
  subscription_ = node->create_subscription<std_msgs::msg::String>("/input", 10, std::bind(&ExtendedInterface::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void ExtendedInterface::topicCallback(const std_msgs::msg::String & msg)
{
  label_->setText(QString(msg.data.c_str()));
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void ExtendedInterface::buttonActivated()
{
    simulation_process_ = new QProcess(this);
    QStringList arguments;
    arguments << "launch" << "common_meta" << "simulation_launch.py";
    simulation_process_->start("ros2", arguments);
}

}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::ExtendedInterface, rviz_common::Panel)