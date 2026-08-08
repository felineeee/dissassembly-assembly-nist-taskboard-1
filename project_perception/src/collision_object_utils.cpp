#include "project_perception/collision_object_utils.hpp"

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>

namespace irb1200_perception
{

moveit_msgs::msg::CollisionObject createCollisionObjectFromSTL(const std::string& object_id,
                                                               const std::string& stl_file_path,
                                                               const geometry_msgs::msg::Pose& pose,
                                                               const std::string& frame_id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = object_id;

  // Create a mesh from the STL file
  shapes::Mesh* m = shapes::createMeshFromResource("file://" + stl_file_path);

  if (!m)
  {
    // In a real application, you might want to log an error here
    return collision_object;
  }

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  // Free the memory
  delete m;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

}  // namespace irb1200_perception
