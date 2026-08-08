#ifndef IRB1200_PERCEPTION_COLLISION_OBJECT_UTILS_HPP
#define IRB1200_PERCEPTION_COLLISION_OBJECT_UTILS_HPP

#include <string>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace irb1200_perception
{

/**
 * @brief Creates a CollisionObject from an STL file.
 * 
 * @param object_id The unique ID for the collision object.
 * @param stl_file_path Absolute path to the .stl file.
 * @param pose The pose of the object in the specified frame.
 * @param frame_id The reference frame for the object (e.g., "base_link").
 * @return moveit_msgs::msg::CollisionObject The populated collision object message.
 */
moveit_msgs::msg::CollisionObject createCollisionObjectFromSTL(
    const std::string& object_id,
    const std::string& stl_file_path,
    const geometry_msgs::msg::Pose& pose,
    const std::string& frame_id);

} // namespace irb1200_perception

#endif // IRB1200_PERCEPTION_COLLISION_OBJECT_UTILS_HPP
