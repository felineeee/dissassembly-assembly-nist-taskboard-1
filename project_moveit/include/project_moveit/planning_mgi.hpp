#ifndef PLANNING_MGI_HPP
#define PLANNING_MGI_HPP

#include <thread>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Handle all of the prototyped motion planning before export it to stages
 */
class PlanningMGI : public rclcpp::Node
{
public:
  PlanningMGI(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void initializeNode();

  // Stage
  void addStage(const std::string& name, std::function<bool()> exec, std::function<void()> viz = nullptr)
  {
    stages_.push_back({ name, exec, viz ? viz : [] {} });
  }
  bool executeTask();

  // Planning
  bool planExecutePose(const geometry_msgs::msg::Pose& pose);

  // Robot
  bool setGripperPosition(double width);
  bool attachObject(moveit_msgs::msg::CollisionObject& co);
  bool detachObject(moveit_msgs::msg::CollisionObject& co);

  // Visualizaton
  void drawTitle(const std::string& text);
  void drawTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

  // Helper
  std::string resolvePath(const std::string& path);
  geometry_msgs::msg::Pose loadPoseFromYAML(const std::string& id, const std::string& yaml_path);

private:
  struct Stage
  {
    std::string name;
    std::function<bool()> execute;
    std::function<void()> visualize;
  };

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  const moveit::core::JointModelGroup* jmg_;

  moveit::planning_interface::PlanningSceneInterface psi_;

  std::vector<Stage> stages_;

  moveit_visual_tools::MoveItVisualToolsPtr vis_tool_;
};

#endif