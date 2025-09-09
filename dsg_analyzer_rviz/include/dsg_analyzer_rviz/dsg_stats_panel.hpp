#ifndef RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
#define RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_

#include <QLabel>
#include <QString>
#include <QSet>
#include <QPushButton>
#include <QComboBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace dsg_analyzer_rviz
{
class DsgStatsPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit DsgStatsPanel(QWidget * parent = 0);
  ~DsgStatsPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void topicCallback(const std_msgs::msg::String& msg, std::string const &robot_id);

  // map of robot ids to dsg subscribers
  std::map<std::string,
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>
  dsg_stats_subscribers_;

  // Data members
  QSet<QString> robot_ids_;

  QLabel * label_;
  QPushButton * button_;
  QComboBox * p_dsg_stats_robot_id_combo_box;

private Q_SLOTS:
  void buttonActivated();
};

}  // namespace rviz_panel_tutorial

#endif  // RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
