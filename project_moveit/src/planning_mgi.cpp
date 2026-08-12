#include "project_moveit/planning_mgi.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PlanningMGI::PlanningMGI(const rclcpp::NodeOptions& options) : Node("move_group_interface", options)
{
}

void PlanningMGI::initializeNode()
{
  move_group_interface_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "irb120_arm");
  jmg_ = move_group_interface_->getRobotModel()->getJointModelGroup("irb120_arm");

  vis_tool_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface_->getRobotModel());

  vis_tool_->deleteAllMarkers();
  vis_tool_->loadRemoteControl();

  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
  RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
  std::copy(move_group_interface_->getJointModelGroupNames().begin(),
            move_group_interface_->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
}

bool PlanningMGI::executeTask()
{
  for (auto& stage : stages_)
  {
    RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());

    stage.visualize();
    if (!stage.execute())
    {
      RCLCPP_ERROR(this->get_logger(), "Stage '%s' FAILED", stage.name.c_str());
      return false;
    }
  }
  return true;
}

bool PlanningMGI::planExecutePose(const geometry_msgs::msg::Pose& pose)
{
  if (!move_group_interface_)
  {
    RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
    return false;
  }

  move_group_interface_->setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface_->plan(plan));

  if (success)
  {
    if (vis_tool_ && jmg_)
    {
      vis_tool_->publishTrajectoryLine(plan.trajectory, jmg_);
      vis_tool_->trigger();
    }
    move_group_interface_->execute(plan);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
  }

  return success;
}

bool PlanningMGI::setGripperPosition(double width)
{
  const double MIN_CLOSE = 0.001;
  const double MAX_CLOSE = 0.038;
  width = std::clamp(width, MIN_CLOSE, MAX_CLOSE);

  if (!move_group_interface_)
  {
    RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
    return false;
  }

  std::map<std::string, double> joint_goals;
  joint_goals["gripper_gripper_joint"] = width;
  move_group_interface_->setJointValueTarget(joint_goals);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

  if (success)
  {
    success = move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return success;
}

bool PlanningMGI::attachObject(moveit_msgs::msg::CollisionObject& co)
{
  if (!move_group_interface_)
  {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized");
    return false;
  }

  std::vector<std::string> touching_links;
  touching_links.push_back("left_onrobot_2fg7_finger_link");
  touching_links.push_back("right_onrobot_2fg7_finger_link");

  bool success = move_group_interface_->attachObject(co.id, "gripper_base_link", touching_links);

  if (success)
  {
    RCLCPP_INFO(get_logger(), "Attached '%s' to gripper", co.id.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to attach '%s'", co.id.c_str());
  }

  return success;
}

bool PlanningMGI::detachObject(moveit_msgs::msg::CollisionObject& co)
{
  if (!move_group_interface_)
  {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized");
    return false;
  }

  bool success = move_group_interface_->detachObject(co.id);

  if (success)
  {
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi_.applyCollisionObjects({ co });
    RCLCPP_INFO(get_logger(), "Detached '%s' to world", co.id.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to detach '%s'", co.id.c_str());
  }

  return success;
}

void PlanningMGI::drawTitle(const std::string& text)
{
  if (!vis_tool_)
    return;
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  vis_tool_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  vis_tool_->trigger();
}

void PlanningMGI::drawTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  if (vis_tool_ && jmg_)
  {
    vis_tool_->publishTrajectoryLine(trajectory, jmg_);
    vis_tool_->trigger();
  }
}

std::string PlanningMGI::resolvePath(const std::string& path)
{
  const std::string prefix = "project_description://";
  if (path.find(prefix) == 0)
  {
    std::string rel_path = path.substr(prefix.length());
    std::string share_dir = ament_index_cpp::get_package_share_directory("project_description");
    return share_dir + "/" + rel_path;
  }
  return path;
}

// DEFINITION OF loadPoseFromYAML
geometry_msgs::msg::Pose PlanningMGI::loadPoseFromYAML(const std::string& id, const std::string& yaml_path)
{
  std::string full_path = resolvePath(yaml_path);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(full_path);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file '%s': %s", full_path.c_str(), e.what());
    return geometry_msgs::msg::Pose();
  }

  if (!config["objects"])
  {
    RCLCPP_WARN(this->get_logger(), "No objects found in YAML file '%s'", full_path.c_str());
    return geometry_msgs::msg::Pose();
  }

  geometry_msgs::msg::Pose pose;

  for (const auto& obj : config["objects"])
  {
    if (obj["id"] && obj["id"].as<std::string>() == id && obj["Pose"])
    {
      std::vector<double> pose_array = obj["Pose"].as<std::vector<double>>();
      if (pose_array.size() < 6)
      {
        RCLCPP_ERROR(this->get_logger(), "Pose array too short: %zu", pose_array.size());
        return geometry_msgs::msg::Pose();
      }
      pose.position.x = pose_array[0];
      pose.position.y = pose_array[1];
      pose.position.z = pose_array[2];

      tf2::Quaternion q;
      q.setRPY(pose_array[3], pose_array[4], pose_array[5]);

      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      return pose;
    }
  }

  RCLCPP_WARN(this->get_logger(), "Pose not found for ID: %s in %s", id.c_str(), full_path.c_str());
  return geometry_msgs::msg::Pose();
}