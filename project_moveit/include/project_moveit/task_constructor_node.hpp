#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "project_moveit/mtc_containers.hpp"

namespace irb120_assembly
{

class TaskConstructor : public rclcpp::Node
{
public:
  explicit TaskConstructor(const rclcpp::NodeOptions& options);
  void doTask();

private:
  RobotConfig config_;
  Solvers solvers_;
  mtc::Task task_;
  mtc::Stage* current_state_ptr_ = nullptr;

  mtc::Task createTask();
  void setupEnvironment(mtc::Task& task);

  // --- Sub-task Builders for Part Families ---
  void addNutTasks(mtc::Task& task);
  void addKetTasks(mtc::Task& task);
  void addRcogTasks(mtc::Task& task);
  void addConnectorTasks(mtc::Task& task);
  void addGearTasks(mtc::Task& task);
};

}  // namespace irb120_assembly