#ifndef SCENE_LOADER_HPP
#define SCENE_LOADER_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include "project_interfaces/srv/scene_load.hpp"
/**
 * @brief ros2 service call /load_planning_scene project_interfaces/srv/SceneLoad "{scene_yaml_path:
 * '/path/to/your/scene.yaml'}"
 */
class SceneLoader : public rclcpp::Node
{
public:
  SceneLoader(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool spawnScene(const std::string& yaml_path);
  void spawnScene(const std::shared_ptr<project_interfaces::srv::SceneLoad::Request> req,
                  std::shared_ptr<project_interfaces::srv::SceneLoad::Response> res);
  void spawnDebugBox();
  std::string resolvePath(const std::string& path);

  rclcpp::Service<project_interfaces::srv::SceneLoad>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  moveit::planning_interface::PlanningSceneInterface psi_;
};
#endif
