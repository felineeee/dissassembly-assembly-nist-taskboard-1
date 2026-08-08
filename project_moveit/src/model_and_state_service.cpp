#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
/**
 * @note use mutex for thread safety
 * parameter handline
 * smart pointer for large object
 */
class ModelState : public rclcpp::Node
{
public:
  ModelState(const rclcpp::NodeOptions& options) : Node("model_state", options)
  {
  }

  void initialize()
  {
    robot_model_loader::RobotModelLoader::Options loader_options("robot_description");
    model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(shared_from_this(), loader_options);
    kinematic_model_ = model_loader_->getModel();

    if (!kinematic_model_)
    {
      RCLCPP_ERROR(this->get_logger(), "Robot model not loaded.");
      return;
    }

    robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    robot_state_->setToDefaultValues();

    arm_jmg_ = kinematic_model_->getJointModelGroup("irb120_arm");
    gripper_jmg_ = kinematic_model_->getJointModelGroup("irb12_gripper");

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->onTimerTick(); });
  }

  /*
  * @todo fix this later
  // Get root link (not model frame)
  std::string root_link = kinematic_model_->getRootLink()->getName();  // Usually "base_link"

  // Publish root link relative to model frame (world/base)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = kinematic_model_->getModelFrame();
    tf.child_frame_id = root_link;
    // ... fill transform ...
    transforms.push_back(tf);
  }

  // Publish child links relative to parents
  for (const auto& link_name : link_names) {
    if (link_name == root_link) continue;  // Skip root (handled above)

    std::string parent_name = kinematic_model_->getLinkModel(link_name)->getParentLinkModel()->getName();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = parent_name;
    tf.child_frame_id = link_name;
    // ... fill from robot_state_->getLinkTransform(link_name, parent_name) ...
    transforms.push_back(tf);
  }

  */
private:
  void onTimerTick()
  {
    robot_state_->update();

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    moveit::core::robotStateToJointStateMsg(*robot_state_, joint_msg);
    joint_pub_->publish(joint_msg);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    rclcpp::Time now = this->now();

    const std::vector<std::string>& link_names = kinematic_model_->getLinkModelNames();

    for (const auto& link_name : link_names)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = kinematic_model_->getModelFrame();
      tf.child_frame_id = link_name;

      if (tf.header.frame_id == tf.child_frame_id)
      {
        continue;
      }

      const Eigen::Isometry3d& global_transform = robot_state_->getGlobalLinkTransform(link_name);

      tf.transform.translation.x = global_transform.translation().x();
      tf.transform.translation.y = global_transform.translation().y();
      tf.transform.translation.z = global_transform.translation().z();

      Eigen::Quaterniond q(global_transform.rotation());
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      transforms.push_back(tf);
    }
    tf_broadcaster_->sendTransform(transforms);

    // DEBUG, turning off for now
    // logModelState();
  }

  void logModelState()
  {
    std::vector<double> joint_values;
    if (arm_jmg_)
    {
      robot_state_->copyJointGroupPositions(arm_jmg_, joint_values);
      auto names = arm_jmg_->getVariableNames();
      for (size_t i = 0; i < names.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "Joint %s: %f", names[i].c_str(), joint_values[i]);
      }
    }
    RCLCPP_INFO(this->get_logger(), "==============================\n");
  }

  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* arm_jmg_ = nullptr;
  const moveit::core::JointModelGroup* gripper_jmg_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ModelState>(node_options);
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}