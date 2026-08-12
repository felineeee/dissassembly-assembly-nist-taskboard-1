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
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
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

class SceneLoaderInterface : public rclcpp::Node
{
public:
  explicit SceneLoaderInterface(const rclcpp::Node::SharedPtr& node, const std::string& planning_group = "panda_group");

private:
};

class SceneLoaderApi : public rclcpp::Node
{
public:
  explicit SceneLoaderApi(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void initVisualTools();
  void loadAssemblyEnvironment();
  void loadAssemblyEnvironmentFromUrdf(const std::string& urdf_file_path);
  void runTutorial();

private:
  // Example
  moveit_msgs::msg::AttachedCollisionObject createBox();
  void publishSceneDiff(const moveit_msgs::msg::CollisionObject& obj);
  void applySceneViaService(const moveit_msgs::msg::CollisionObject& obj);
  void attachObject(const moveit_msgs::msg::AttachedCollisionObject& obj);
  void detachObject(const moveit_msgs::msg::AttachedCollisionObject& obj);
  void removeObjectFromWorld(const std::string& id);

  // NIST Taskboard 1
  moveit_msgs::msg::CollisionObject createMeshObject(const std::string& id, const std::string& frame_id,
                                                     const std::string& mesh_resource_path,
                                                     const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Vector3d& scale = Eigen::Vector3d(1.0, 1.0, 1.0));
  void publishBatchSceneDiff(const std::vector<moveit_msgs::msg::CollisionObject>& objs);

  // Members
  std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_pub_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
};

#endif
