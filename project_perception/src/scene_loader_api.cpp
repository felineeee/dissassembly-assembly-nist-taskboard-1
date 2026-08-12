#include "project_perception/scene_loader.hpp"
#include <chrono>
#include <shape_msgs/msg/solid_primitive.hpp>

SceneLoaderApi::SceneLoaderApi(const rclcpp::NodeOptions& options) : Node("scene_loader_api_node", options)
{
  // 1. Setup ROS Interfaces
  // Publisher for asynchronous updates
  planning_scene_diff_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  // Client for synchronous updates (wait for confirmation)
  apply_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

  // Wait for subscribers/service
  while (planning_scene_diff_pub_->get_subscription_count() < 1)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for planning scene subscribers...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
}

void SceneLoaderApi::initVisualTools()
{
  visual_tools_ = std::make_unique<rviz_visual_tools::RvizVisualTools>("base_link", "planning_scene_ros_api_tutorial",
                                                                       shared_from_this());
  visual_tools_->loadRemoteControl();
  visual_tools_->deleteAllMarkers();
}

void SceneLoaderApi::runTutorial()
{
  // Define a box object for the demo
  moveit_msgs::msg::AttachedCollisionObject box = createBox();

  // --- STEP 1: Add object to world ---
  visual_tools_->prompt("Next: Add object to world");
  publishSceneDiff(box.object);

  // --- STEP 2: Synchronous update via Service ---
  visual_tools_->prompt("Next: Add object via Service (Synchronous)");
  applySceneViaService(box.object);

  // --- STEP 3: Attach object to robot ---
  visual_tools_->prompt("Next: Attach object to robot hand");
  attachObject(box);

  // --- STEP 4: Detach object ---
  visual_tools_->prompt("Next: Detach object and return to world");
  detachObject(box);

  // --- STEP 5: Remove from world ---
  visual_tools_->prompt("Next: Remove object from world entirely");
  removeObjectFromWorld(box.object.id);

  RCLCPP_INFO(this->get_logger(), "Tutorial Complete!");
}

moveit_msgs::msg::AttachedCollisionObject SceneLoaderApi::createBox()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";
  attached_object.object.header.frame_id = "panda_hand";
  attached_object.object.id = "box";

  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = { 0.075, 0.075, 0.075 };

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.object.operation = attached_object.object.ADD;

  // Allow the box to touch the hand without triggering collision
  attached_object.touch_links = { "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  return attached_object;
}

void SceneLoaderApi::publishSceneDiff(const moveit_msgs::msg::CollisionObject& obj)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects.push_back(obj);
  planning_scene_diff_pub_->publish(scene);
}

void SceneLoaderApi::applySceneViaService(const moveit_msgs::msg::CollisionObject& obj)
{
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  request->scene.world.collision_objects.push_back(obj);

  if (!apply_scene_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return;
  }

  auto result = apply_scene_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Service call successful");
  }
}

void SceneLoaderApi::attachObject(const moveit_msgs::msg::AttachedCollisionObject& obj)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  // To attach: 1. Remove from world, 2. Add to RobotState
  moveit_msgs::msg::CollisionObject remove_cmd;
  remove_cmd.id = obj.object.id;
  remove_cmd.operation = remove_cmd.REMOVE;

  scene.world.collision_objects.push_back(remove_cmd);
  scene.robot_state.attached_collision_objects.push_back(obj);
  scene.robot_state.is_diff = true;

  planning_scene_diff_pub_->publish(scene);
}

void SceneLoaderApi::detachObject(const moveit_msgs::msg::AttachedCollisionObject& obj)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  // To detach: 1. Remove from RobotState, 2. Add back to world
  moveit_msgs::msg::AttachedCollisionObject detach_cmd = obj;
  detach_cmd.object.operation = detach_cmd.object.REMOVE;

  scene.robot_state.attached_collision_objects.push_back(detach_cmd);
  scene.robot_state.is_diff = true;
  scene.world.collision_objects.push_back(obj.object);

  planning_scene_diff_pub_->publish(scene);
}

void SceneLoaderApi::removeObjectFromWorld(const std::string& id)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::CollisionObject remove_cmd;
  remove_cmd.id = id;
  remove_cmd.operation = remove_cmd.REMOVE;
  scene.world.collision_objects.push_back(remove_cmd);
  planning_scene_diff_pub_->publish(scene);
}

// NIST Taskboard 1
moveit_msgs::msg::CollisionObject SceneLoaderApi::createMeshObject(const std::string& id, const std::string& frame_id,
                                                                   const std::string& mesh_resource_path,
                                                                   const geometry_msgs::msg::Pose& pose,
                                                                   const Eigen::Vector3d& scale)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;

  // 1. Parse the .dae / .stl mesh file
  shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource_path, scale);
  if (!m)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load mesh resource: %s", mesh_resource_path.c_str());
    return obj;
  }

  // 2. Convert geometric_shapes Mesh into ROS shape_msgs Mesh
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  delete m;  // Clean up memory

  shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  // 3. Assemble CollisionObject
  obj.meshes.push_back(mesh);
  obj.mesh_poses.push_back(pose);
  obj.operation = obj.ADD;

  return obj;
}

void SceneLoaderApi::loadAssemblyEnvironment()
{
  visual_tools_->prompt("Next: Spawn Taskboard, Tray, and Parts into Planning Scene");

  std::vector<moveit_msgs::msg::CollisionObject> objects_to_add;

  // --- 1. Load NIST Taskboard (.dae mesh) ---
  geometry_msgs::msg::Pose board_pose;
  board_pose.position.x = 0.55;
  board_pose.position.y = 0.15;
  board_pose.position.z = 0.0;
  board_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject taskboard = createMeshObject(
      "nist_taskboard", "panda_link0", "package://my_moveit_package/meshes/taskboard1.dae", board_pose);
  objects_to_add.push_back(taskboard);

  // --- 2. Load Part Tray (.dae mesh) ---
  geometry_msgs::msg::Pose tray_pose;
  tray_pose.position.x = 0.55;
  tray_pose.position.y = -0.25;
  tray_pose.position.z = 0.0;
  tray_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject tray =
      createMeshObject("part_tray", "panda_link0", "package://my_moveit_package/meshes/tray.dae", tray_pose);
  objects_to_add.push_back(tray);

  // --- 3. Load Peg Part inside the Tray (.dae mesh) ---
  // Coordinates relative to panda_link0 or tray
  geometry_msgs::msg::Pose peg_pose;
  peg_pose.position.x = 0.55;
  peg_pose.position.y = -0.25;
  peg_pose.position.z = 0.03;  // Slightly elevated inside tray
  peg_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject peg_part =
      createMeshObject("peg_part_1", "panda_link0", "package://my_moveit_package/meshes/peg.dae", peg_pose);
  objects_to_add.push_back(peg_part);

  // --- 4. Publish Batch Scene Diff over ROS API ---
  publishBatchSceneDiff(objects_to_add);

  RCLCPP_INFO(this->get_logger(), "Taskboard, Tray, and Assembly Parts published to Planning Scene!");
}

void SceneLoaderApi::loadAssemblyEnvironmentFromUrdf(const std::string& urdf_file_path)
{
  urdf::Model model;
  if (!model.initFile(urdf_file_path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file: %s", urdf_file_path.c_str());
    return;
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects_to_add;

  for (const auto& [joint_name, joint_ptr] : model.joints_)
  {
    std::string child_link_name = joint_ptr->child_link_name;
    auto child_link_ptr = model.getLink(child_link_name);

    if (!child_link_ptr)
      continue;

    if (child_link_ptr->visual && child_link_ptr->visual->geometry &&
        child_link_ptr->visual->geometry->type == urdf::Geometry::MESH)
    {
      auto mesh_geom = std::static_pointer_cast<urdf::Mesh>(child_link_ptr->visual->geometry);
      std::string mesh_path = mesh_geom->filename;

      // Force target frame to base_link
      std::string target_frame = "base_link";

      // Direct position mapping
      geometry_msgs::msg::Pose pose;
      pose.position.x = joint_ptr->parent_to_joint_origin_transform.position.x;
      pose.position.y = joint_ptr->parent_to_joint_origin_transform.position.y;
      pose.position.z = joint_ptr->parent_to_joint_origin_transform.position.z;

      // Direct quaternion copy (fixes rotational alignment gap)
      pose.orientation.x = joint_ptr->parent_to_joint_origin_transform.rotation.x;
      pose.orientation.y = joint_ptr->parent_to_joint_origin_transform.rotation.y;
      pose.orientation.z = joint_ptr->parent_to_joint_origin_transform.rotation.z;
      pose.orientation.w = joint_ptr->parent_to_joint_origin_transform.rotation.w;

      Eigen::Vector3d scale(mesh_geom->scale.x, mesh_geom->scale.y, mesh_geom->scale.z);

      moveit_msgs::msg::CollisionObject obj = createMeshObject(child_link_name, target_frame, mesh_path, pose, scale);

      objects_to_add.push_back(obj);
    }
  }

  publishBatchSceneDiff(objects_to_add);
  RCLCPP_INFO(this->get_logger(), "Successfully spawned %zu environment objects relative to base_link!",
              objects_to_add.size());
}

void SceneLoaderApi::publishBatchSceneDiff(const std::vector<moveit_msgs::msg::CollisionObject>& objs)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects = objs;
  planning_scene_diff_pub_->publish(scene);
}