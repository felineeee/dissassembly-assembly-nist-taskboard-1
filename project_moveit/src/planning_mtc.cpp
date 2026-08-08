#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mtc = moveit::task_constructor;

class TaskConstructor : public rclcpp::Node
{
public:
  TaskConstructor(const rclcpp::NodeOptions& options) : Node("task_constructor", options)
  {
  }

  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("M16 Nut Assembly Task");
    task.loadRobotModel(shared_from_this());

    // -------------------------------------------------------------------------
    // 1. Group & Link Definitions matching your URDF
    // -------------------------------------------------------------------------
    const std::string arm_group_name = "irb120_arm";     // Match group name in irb120_gripper.srdf
    const std::string hand_group_name = "onrobot_2fg7";  // Match gripper group name in SRDF
    const std::string eef_frame = "tool0_link";          // EEF parent frame from your URDF
    const std::string target_object = "m16_nut_link";    // Target nut link name from URDF

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", eef_frame);

    // -------------------------------------------------------------------------
    // 2. Solvers Setup
    // -------------------------------------------------------------------------
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.001);  // 1mm precision for insertion

    // Current State Stage
    mtc::Stage* current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // Open Gripper Stage
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open_gripper", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    // Connect to Pick
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move_to_pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    // -------------------------------------------------------------------------
    // 3. Serial Container: PICK NUT
    // -------------------------------------------------------------------------
    mtc::Stage* attach_object_stage = nullptr;
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick_nut_container");
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      // Approach Nut along EEF Z axis
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_nut", cartesian_planner);
        stage->properties().set("marker_ns", "approach_nut");
        stage->properties().set("link", eef_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.15);

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = eef_frame;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      // Generate Grasp Pose over m16_nut_link
      {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject(target_object);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);

        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.translation().z() = 0.05;  // 5cm offset above nut center

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp_pose_IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, eef_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }

      // Allow Collisions between 2FG7 fingers & Nut
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision_finger_nut");
        std::vector<std::string> finger_links = { "gripper_left_attachment", "gripper_right_attachment" };
        stage->allowCollisions(target_object, finger_links, true);
        grasp->insert(std::move(stage));
      }

      // Close Gripper
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close_gripper", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("close");
        grasp->insert(std::move(stage));
      }

      // Attach Object to Gripper Base Link
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_nut_to_gripper");
        stage->attachObject(target_object, eef_frame);
        attach_object_stage = stage.get();
        grasp->insert(std::move(stage));
      }

      // Lift Object
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift_nut", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.2);
        stage->setIKFrame(eef_frame);
        stage->properties().set("marker_ns", "lift_nut");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "base_link";
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      task.add(std::move(grasp));
    }

    // -------------------------------------------------------------------------
    // 4. Serial Container: PLACE / INSERT NUT ON TASKBOARD
    // -------------------------------------------------------------------------
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move_to_taskboard", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                                       { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));

    {
      auto place = std::make_unique<mtc::SerialContainer>("place_nut_container");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      // Target Insertion Pose on Taskboard
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate_assembly_pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(target_object);

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "base_link";
        target_pose_msg.pose.position.x = -0.200;  // Over taskboard bolt
        target_pose_msg.pose.position.y = -0.315;
        target_pose_msg.pose.position.z = 0.050;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(attach_object_stage);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("assembly_pose_IK", std::move(stage));
        wrapper->setMaxIKSolutions(4);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(target_object);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }

      // Open Gripper
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper_release", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        place->insert(std::move(stage));
      }

      // Detach Object back to World / base_link
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_nut_to_world");
        stage->detachObject(target_object, eef_frame);
        place->insert(std::move(stage));
      }

      // Retract Arm Straight Up
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retract_arm", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.2);
        stage->setIKFrame(eef_frame);
        stage->properties().set("marker_ns", "retract");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "base_link";
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      task.add(std::move(place));
    }

    return task;
  }

  void doTask()
  {
    task_ = createTask();
    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), e);
      return;
    }

    if (!task_.plan(5))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed!");
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Assembly task executed successfully!");
  }

private:
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<TaskConstructor>(node_options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(mtc_task_node->get_node_base_interface());
  });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}