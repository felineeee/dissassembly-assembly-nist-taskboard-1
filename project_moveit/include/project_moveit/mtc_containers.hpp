#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

namespace mtc = moveit::task_constructor;

namespace irb120_assembly
{

// Robot Frame and Group Configuration
struct RobotConfig
{
  std::string arm_group = "irb1200_arm";
  std::string hand_group = "robotiq_gripper";
  std::string eef_frame = "robotiq_tcp";
  std::string base_frame = "base_link";
};

// Planners container
struct Solvers
{
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian;
};

// --- MTC Factory Helpers ---

std::unique_ptr<mtc::stages::Connect> createConnectStage(const std::string& name, const std::string& arm_group,
                                                         std::shared_ptr<mtc::solvers::PipelinePlanner> planner);

std::unique_ptr<mtc::SerialContainer> createPickContainer(mtc::Task& task, const RobotConfig& config,
                                                          const std::string& part_name,
                                                          const Eigen::Isometry3d& grasp_tf, const Solvers& solvers,
                                                          mtc::Stage* monitored_stage, mtc::Stage** attach_stage_out);

std::unique_ptr<mtc::SerialContainer> createPlaceContainer(mtc::Task& task, const RobotConfig& config,
                                                           const std::string& part_name,
                                                           const geometry_msgs::msg::PoseStamped& target_pose,
                                                           const Solvers& solvers, mtc::Stage* attach_stage,
                                                           const std::string& surface_link = "taskboard_link");

std::unique_ptr<mtc::SerialContainer> createFastenContainer(mtc::Task& task, const RobotConfig& config,
                                                            const std::string& part_name,
                                                            const geometry_msgs::msg::PoseStamped& target_pose,
                                                            const Solvers& solvers, mtc::Stage* attach_stage,
                                                            double twist_angle_rad, double thread_pitch,
                                                            const std::string& surface_link = "taskboard_link");

std::unique_ptr<mtc::SerialContainer> createMeshGearContainer(mtc::Task& task, const RobotConfig& config,
                                                              const std::string& part_name,
                                                              const geometry_msgs::msg::PoseStamped& target_pose,
                                                              const Solvers& solvers, mtc::Stage* attach_stage,
                                                              double mesh_angle_rad);

std::unique_ptr<mtc::stages::MoveRelative> createTransit(const RobotConfig& config, const Solvers& solvers,
                                                         const Eigen::Vector3d& translation,
                                                         const std::string& frame_id = "world",
                                                         const std::string& name = "transit");

void restoreCollision(mtc::Task& task, const std::string& part_name, const std::string& target_link,
                      mtc::Stage*& current_state_ptr);

}  // namespace irb120_assembly