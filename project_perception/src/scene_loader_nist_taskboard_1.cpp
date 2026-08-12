#include "project_perception/scene_loader_nist_taskboard_1.hpp"

PlanningSceneNistTaskboard1::PlanningSceneNistTaskboard1(const rclcpp::Logger& node_logger) : logger_(node_logger)
{
}

geometry_msgs::msg::Quaternion PlanningSceneNistTaskboard1::createQuaternionFromRPY(double r, double p, double y)
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  return tf2::toMsg(q);
}

bool PlanningSceneNistTaskboard1::spawnMeshObject(const std::string& object_id, const std::string& package_name,
                                                  const std::string& relative_mesh_path,
                                                  const std::string& parent_frame, double x, double y, double z,
                                                  double roll, double pitch, double yaw)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = parent_frame;
  obj.id = object_id;

  std::string package_share = ament_index_cpp::get_package_share_directory(package_name);
  std::string full_mesh_path = package_share + "/" + relative_mesh_path;

  shapes::Mesh* m = shapes::createMeshFromResource("file://" + full_mesh_path);
  if (!m)
  {
    RCLCPP_ERROR(logger_, "Failed to load mesh: %s", full_mesh_path.c_str());
    return false;
  }

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  delete m;

  shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = createQuaternionFromRPY(roll, pitch, yaw);

  obj.meshes.push_back(mesh);
  obj.mesh_poses.push_back(pose);
  obj.operation = obj.ADD;

  psi_.applyCollisionObject(obj);
  spawned_object_ids_.push_back(object_id);

  RCLCPP_INFO(logger_, "Spawned '%s' relative to '%s'", object_id.c_str(), parent_frame.c_str());
  return true;
}

bool PlanningSceneNistTaskboard1::spawnM16Nut(const std::string& parent_frame, double x, double y, double z,
                                              double roll, double pitch, double yaw)
{
  return spawnMeshObject("m16_nut", "project_description", "meshes/taskboard_1/dae/M16_Nut.dae", parent_frame, x, y, z,
                         roll, pitch, yaw);
}

void PlanningSceneNistTaskboard1::removeObject(const std::string& object_id)
{
  psi_.removeCollisionObjects({ object_id });
}

void PlanningSceneNistTaskboard1::clearAllObjects()
{
  if (!spawned_object_ids_.empty())
  {
    psi_.removeCollisionObjects(spawned_object_ids_);
    spawned_object_ids_.clear();
  }
}
