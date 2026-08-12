#ifndef PLANNING_SCENE_PREP_HPP
#define PLANNING_SCENE_PREP_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PlanningSceneNistTaskboard1
{
public:
  explicit PlanningSceneNistTaskboard1(const rclcpp::Logger& node_logger = rclcpp::get_logger("planning_scene_prep"));

  bool spawnMeshObject(const std::string& object_id, const std::string& package_name,
                       const std::string& relative_mesh_path, const std::string& parent_frame, double x, double y,
                       double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0);

  bool spawnM16Nut(const std::string& parent_frame, double x, double y, double z, double roll = 0.0, double pitch = 0.0,
                   double yaw = 0.0);

  void removeObject(const std::string& object_id);
  void clearAllObjects();

private:
  geometry_msgs::msg::Quaternion createQuaternionFromRPY(double r, double p, double y);

  rclcpp::Logger logger_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  std::vector<std::string> spawned_object_ids_;
};

#endif  // PLANNING_SCENE_PREP_HPP