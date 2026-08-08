#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class CollisionObjectTransform : public rclcpp::Node
{
public:
  CollisionObjectTransform() : Node("collision_object_transform")
  {
    this->declare_parameter<std::string>("target_object", "");
    this->declare_parameter<double>("goal_x", 0.);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<double>("goal_z", 0.0);
    this->declare_parameter<double>("goal_qx", 0.0);
    this->declare_parameter<double>("goal_qy", 0.0);
    this->declare_parameter<double>("goal_qz", 0.0);
    this->declare_parameter<double>("goal_qw", 0.0);
    psi_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { this->transformObject(); });
  }
  void transformObject()
  {
    this->timer_->cancel();

    std::string target_id = this->get_parameter("target_object").as_string();
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = this->get_parameter("goal_x").as_double();
    target_pose.position.y = this->get_parameter("goal_y").as_double();
    target_pose.position.z = this->get_parameter("goal_z").as_double();

    target_pose.orientation.x = this->get_parameter("goal_qx").as_double();
    target_pose.orientation.y = this->get_parameter("goal_qy").as_double();
    target_pose.orientation.z = this->get_parameter("goal_qz").as_double();
    target_pose.orientation.w = this->get_parameter("goal_qw").as_double();

    RCLCPP_INFO(this->get_logger(), "Executing task for object: [%s]", target_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Goal Position: [X: %.3f, Y: %.3f, Z: %.3f]", target_pose.position.x,
                target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Goal Orientation: [QX: %.2f, QY: %.2f, QZ: %.2f, QW: %.2f]",
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,
                target_pose.orientation.w);

    try
    {
      auto objects = psi_->getObjects({ target_id });

      if (objects.count(target_id))
      {
        moveit_msgs::msg::CollisionObject co;
        co.id = target_id;
        co.header.frame_id = "base_link";
        co.pose = target_pose;
        co.operation = co.MOVE;  // ADD acts as an update if the ID already exists

        RCLCPP_INFO(this->get_logger(), "Object Position: [X: %.3f, Y: %.3f, Z: %.3f]", co.pose.position.x,
                    co.pose.position.y, co.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Object Orientation: [QX: %.2f, QY: %.2f, QZ: %.2f, QW: %.2f]",
                    co.pose.orientation.x, co.pose.orientation.y, co.pose.orientation.z, co.pose.orientation.w);

        psi_->applyCollisionObject(co);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_INFO(this->get_logger(), "Planning scene updated successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Object '%s' not found in Scene!", target_id.c_str());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    }
  }

private:
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionObjectTransform>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}