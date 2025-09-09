#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <dsg_analyzer_rviz/dsg_stats_panel.hpp>

namespace dsg_analyzer_rviz
{
DsgStatsPanel::DsgStatsPanel(QWidget * parent) : Panel(parent)
{
  // Create the layout for the approval robot id
  QHBoxLayout *p_dsg_stats_robot_id_combo_box_layout = new QHBoxLayout;
  p_dsg_stats_robot_id_combo_box_layout->addWidget(new QLabel("Robot ID"));
  p_dsg_stats_robot_id_combo_box = new QComboBox;
  p_dsg_stats_robot_id_combo_box_layout->addWidget(
      p_dsg_stats_robot_id_combo_box);

  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("Clear stats");
  layout->addLayout(p_dsg_stats_robot_id_combo_box_layout);
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &DsgStatsPanel::buttonActivated);
}

DsgStatsPanel::~DsgStatsPanel() = default;

void DsgStatsPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  // Get the robot ids & populate the relevant combo boxes
  if (!node->has_parameter("robot_ids")) {
    node->declare_parameter<std::vector<std::string>>("robot_ids", {"global"});
  }
  auto param_robot_ids = node->get_parameter("robot_ids");

  for (auto &s : param_robot_ids.as_string_array()) {
    robot_ids_.insert(s.c_str());
  }

  p_dsg_stats_robot_id_combo_box->addItems(
      QList<QString>(robot_ids_.begin(), robot_ids_.end()));

  // create dsg subscriber for each robot id 
    for (auto const &q_robot_id : robot_ids_) {
        auto const robot_id = q_robot_id.toStdString();
        auto const topic_name = "/" + robot_id + "/dsg_analyzer/dsg_stats";
        RCLCPP_INFO(node->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
        dsg_stats_subscribers_.emplace(
            robot_id, 
            // node->create_subscription<std_msgs::msg::String>(
            //     topic_name, 10, std::bind(&DsgStatsPanel::topicCallback, this, std::placeholders::_1))
            node->create_subscription<std_msgs::msg::String>(
                topic_name, 10, 
                [this, robot_id](std_msgs::msg::String msg) {
                    this->topicCallback(msg, robot_id);
            }));
    }
  publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void DsgStatsPanel::topicCallback(const std_msgs::msg::String& msg, std::string const &robot_id)
{
  // Set the combo box to the robot id
  // p_dsg_stats_robot_id_combo_box->setCurrentText(robot_id.c_str());
  if (p_dsg_stats_robot_id_combo_box->currentText() == robot_id.c_str()) {
    label_->setText(QString(msg.data.c_str()));
  }
  
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void DsgStatsPanel::buttonActivated()
{
//   auto message = std_msgs::msg::String();
//   message.data = "Button clicked!";
//   publisher_->publish(message);
  label_->setText("[no data]");
}

}  // namespace dsg_analyzer_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dsg_analyzer_rviz::DsgStatsPanel, rviz_common::Panel)
