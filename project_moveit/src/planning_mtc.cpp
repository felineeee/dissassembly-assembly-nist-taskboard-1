#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace mtc = moveit::task_constructor;

class TaskConstructor : public rclcpp::Node
{
public:
  TaskConstructor(const rclcpp::NodeOptions& options) : Node("task_constructor", options)
  {
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Initialization failed: \n" << e);
      return;
    }

    if (!task_.plan(5))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed!");
      return;
    }

    if (task_.solutions().empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Task planned successfully, but no solutions were found!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Task planning succeeded! Publishing solution to RViz...");
    auto& first_solution = *task_.solutions().front();
    task_.introspection().publishSolution(first_solution);

    auto execution_result = task_.execute(first_solution);
    if (execution_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Task execution completed successfully!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Task execution failed!");
    }
  }

private:
  static constexpr auto ARM_GROUP = "irb1200_arm";
  static constexpr auto HAND_GROUP = "robotiq_gripper";
  static constexpr auto EEF_FRAME = "robotiq_tcp";
  static constexpr auto BASE_FRAME = "base_link";

  mtc::Task task_;
  mtc::Stage* current_state_ptr_ = nullptr;

  // Struct to easily pass all planners to the helper functions
  struct Solvers
  {
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian;
  };

  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("Explicit Assembly Task");
    task.loadRobotModel(shared_from_this());

    task.setProperty("group", ARM_GROUP);
    task.setProperty("eef", HAND_GROUP);
    task.setProperty("ik_frame", EEF_FRAME);

    // Initialize Planners
    Solvers solvers;
    solvers.sampling = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    solvers.interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    solvers.cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    solvers.cartesian->setMaxVelocityScalingFactor(0.2);
    solvers.cartesian->setMaxAccelerationScalingFactor(0.2);
    solvers.cartesian->setStepSize(0.005);

    // 1. Setup Initial State and Base Collisions
    setupEnvironment(task, solvers);

    // =========================================================================
    // PART 1: EXPLICIT DEFINITION
    // =========================================================================
    /*
     * Fin length is about 0.144m
     * Board height is about 0.009m (9mm)
     * Safe Transit height(so it doesnt knock things of)
     * Regrasp (Place -> Connect -> Pick)
     *
     */

    // NUTS
    /*
     * This types of parts doesnt use place, it uses `fasten` instead
     * Pick-place-fasten
     */
    // M4 Nut
    // Pick -> transit -> approach -> fasten
    const std::string PART_1_NAME = "m4_nut_link";

    Eigen::Isometry3d m4_nut_grasp_tf = Eigen::Isometry3d::Identity();
    // Use this line if you need tilt, or remove it for straight-down grasp
    m4_nut_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    m4_nut_grasp_tf.translation().z() = 0.1118;  // 0.115 - 0.0032

    geometry_msgs::msg::PoseStamped m4_nut_place_pose;
    m4_nut_place_pose.header.frame_id = "taskboard_link";
    m4_nut_place_pose.pose.position.x = 0.19255;
    m4_nut_place_pose.pose.position.y = -0.12218;
    m4_nut_place_pose.pose.position.z = 0.187;  // 16mm - 9mm + 180mm

    m4_nut_place_pose.pose.orientation.x = 0.0;
    m4_nut_place_pose.pose.orientation.y = 1.0;
    m4_nut_place_pose.pose.orientation.z = 0.0;
    m4_nut_place_pose.pose.orientation.w = 0.0;

    double m4_thread_pitch = 0.0007;    // m4 coarse pitch 0.7mm
    double m4_exposed_height = 0.0007;  // 7mm exposed, 16mm-9mm
    double m4_twist_angle = (m4_exposed_height / m4_thread_pitch) * (2.0 * M_PI);

    mtc::Stage* m4_attach_nut_stage = nullptr;

    // Pick
    task.add(createConnectStage("connect_to_pick_" + PART_1_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_1_NAME, m4_nut_grasp_tf, solvers, current_state_ptr_, &m4_attach_nut_stage));

    // Transit (Safe)
    task.add(createConnectStage("connect_to_transit_" + PART_1_NAME, solvers.sampling));
    task.add(createTransit(solvers, Eigen::Vector3d(0.0, 0.0, 0.15)));
    // Fasten
    task.add(createConnectStage("connect_to_fasten_" + PART_1_NAME, solvers.sampling));
    task.add(createFastenContainer(task, PART_1_NAME, m4_nut_place_pose, solvers, m4_attach_nut_stage, m4_twist_angle,
                                   m4_thread_pitch));

    // Restore environment collision after part 1 is done, so we can pick part 2
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_1_NAME);
      stage->allowCollisions(PART_1_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // M8 Nut
    const std::string PART_2_NAME = "m8_nut_link";

    Eigen::Isometry3d m8_nut_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp facing straight down
    m8_nut_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    m8_nut_grasp_tf.translation().z() = 0.1085;  // 0.115m finger origin - 0.0065m M8 nut height

    geometry_msgs::msg::PoseStamped m8_nut_place_pose;
    m8_nut_place_pose.header.frame_id = "taskboard_link";
    m8_nut_place_pose.pose.position.x = -0.04145;
    m8_nut_place_pose.pose.position.y = -0.04718;
    m8_nut_place_pose.pose.position.z = 0.189;  // 18mm - 9mm + 180mm

    // Top-down orientation pointing straight down
    m8_nut_place_pose.pose.orientation.x = 0.0;
    m8_nut_place_pose.pose.orientation.y = 1.0;
    m8_nut_place_pose.pose.orientation.z = 0.0;
    m8_nut_place_pose.pose.orientation.w = 0.0;

    double m8_thread_pitch = 0.00125;   // m4 coarse pitch 0.7mm
    double m8_exposed_height = 0.0009;  // 9mm exposed, 18mm-9mm
    double m8_twist_angle = (m8_exposed_height / m8_thread_pitch) * (2.0 * M_PI);

    mtc::Stage* m8_attach_nut_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_2_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_2_NAME, m8_nut_grasp_tf, solvers, current_state_ptr_, &m8_attach_nut_stage));

    // Transit (Safe)
    task.add(createConnectStage("connect_to_transit_" + PART_2_NAME, solvers.sampling));
    task.add(createTransit(solvers, Eigen::Vector3d(0.0, 0.0, 0.15)));

    // Fasten
    task.add(createConnectStage("connect_to_fasten_" + PART_2_NAME, solvers.sampling));
    task.add(createFastenContainer(task, PART_2_NAME, m8_nut_place_pose, solvers, m8_attach_nut_stage, m8_twist_angle,
                                   m8_thread_pitch));

    // Restore environment collision after part 2 is done, so we can pick part 3
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_2_NAME);
      stage->allowCollisions(PART_2_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // M12 Nut
    const std::string PART_3_NAME = "m12_nut_link";

    Eigen::Isometry3d m12_nut_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp facing straight down
    m12_nut_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    m12_nut_grasp_tf.translation().z() = 0.1042;  // 0.115m finger origin - 0.0108m M12 nut height

    geometry_msgs::msg::PoseStamped m12_nut_place_pose;
    m12_nut_place_pose.header.frame_id = "taskboard_link";
    m12_nut_place_pose.pose.position.x = 0.11745;
    m12_nut_place_pose.pose.position.y = 0.03682;
    m12_nut_place_pose.pose.position.z = 0.191;  // 20mm - 9mm + 180mm = 191mm = 0.191m

    // Top-down orientation pointing straight down
    m12_nut_place_pose.pose.orientation.x = 0.0;
    m12_nut_place_pose.pose.orientation.y = 1.0;
    m12_nut_place_pose.pose.orientation.z = 0.0;
    m12_nut_place_pose.pose.orientation.w = 0.0;

    double m12_thread_pitch = 0.00175;   // m4 coarse pitch 0.7mm
    double m12_exposed_height = 0.0011;  // 11mm exposed, 20mm-9mm
    double m12_twist_angle = (m12_exposed_height / m12_thread_pitch) * (2.0 * M_PI);

    mtc::Stage* m12_attach_nut_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_3_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_3_NAME, m12_nut_grasp_tf, solvers, current_state_ptr_, &m12_attach_nut_stage));

    // Transit (Safe)
    task.add(createConnectStage("connect_to_transit_" + PART_3_NAME, solvers.sampling));
    task.add(createTransit(solvers, Eigen::Vector3d(0.0, 0.0, 0.15)));

    // Fasten
    task.add(createConnectStage("connect_to_fasten_" + PART_3_NAME, solvers.sampling));
    task.add(createFastenContainer(task, PART_3_NAME, m12_nut_place_pose, solvers, m12_attach_nut_stage,
                                   m12_twist_angle, m12_thread_pitch));

    // Restore environment collision after part 3 is done, so we can pick part 4
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_3_NAME);
      stage->allowCollisions(PART_3_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // M16 Nut
    const std::string PART_4_NAME = "m16_nut_link";

    Eigen::Isometry3d m16_nut_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways pitch (M_PI / 2.0) + Yaw rotation (+90 degrees / 1.5708 rad)
    m16_nut_grasp_tf.linear() = (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()))
                                    .toRotationMatrix();
    m16_nut_grasp_tf.translation().z() = 0.1002;  // 0.115m finger origin - 0.0148m M16 nut height/half-width

    geometry_msgs::msg::PoseStamped m16_nut_place_pose;
    m16_nut_place_pose.header.frame_id = "taskboard_link";
    m16_nut_place_pose.pose.position.x = 0.04255;
    m16_nut_place_pose.pose.position.y = -0.04718;
    m16_nut_place_pose.pose.position.z = 0.196;  // 25mm bolt - 9mm board + 180mm = 196mm = 0.196m

    // Rotated +90 deg around Z for 180 -> 270 deg transition facing
    // Quaternion for (Pitch=180 deg, Yaw=+90 deg)
    m16_nut_place_pose.pose.orientation.x = 0.7071;
    m16_nut_place_pose.pose.orientation.y = 0.7071;
    m16_nut_place_pose.pose.orientation.z = 0.0;
    m16_nut_place_pose.pose.orientation.w = 0.0;

    double m16_thread_pitch = 0.002;     // m4 coarse pitch 0.7mm
    double m16_exposed_height = 0.0016;  // 16mm exposed, 25mm-9mm
    double m16_twist_angle = (m16_exposed_height / m16_thread_pitch) * (2.0 * M_PI);

    mtc::Stage* m16_attach_nut_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_4_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_4_NAME, m16_nut_grasp_tf, solvers, current_state_ptr_, &m16_attach_nut_stage));

    // Transit (Safe)
    task.add(createConnectStage("connect_to_transit_" + PART_4_NAME, solvers.sampling));
    task.add(createTransit(solvers, Eigen::Vector3d(0.0, 0.0, 0.15)));

    // Fasten
    task.add(createConnectStage("connect_to_fasten_" + PART_4_NAME, solvers.sampling));
    task.add(createFastenContainer(task, PART_4_NAME, m16_nut_place_pose, solvers, m16_attach_nut_stage,
                                   m16_twist_angle, m16_thread_pitch));

    // Restore environment collision after part 4 is done, so we can pick part 5
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_4_NAME);
      stage->allowCollisions(PART_4_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // KET
    // KET 4
    const std::string PART_5_NAME = "ket4_link";

    Eigen::Isometry3d ket4_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp facing straight down
    ket4_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    ket4_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped ket4_place_pose;
    ket4_place_pose.header.frame_id = "taskboard_link";
    ket4_place_pose.pose.position.x = 0.19255;    // 192.55 mm -> 0.19255 m
    ket4_place_pose.pose.position.y = -0.197176;  // -197.176 mm -> -0.197176 m
    ket4_place_pose.pose.position.z = 0.191;      // Taskboard height

    // Top-down orientation, Y axis facing (180 deg rotation around Y axis)
    ket4_place_pose.pose.orientation.x = 0.0;
    ket4_place_pose.pose.orientation.y = 1.0;
    ket4_place_pose.pose.orientation.z = 0.0;
    ket4_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* ket4_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_5_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_5_NAME, ket4_grasp_tf, solvers, current_state_ptr_, &ket4_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_5_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_5_NAME, ket4_place_pose, solvers, ket4_attach_stage));

    // Restore environment collision after part 4 is done
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_5_NAME);
      stage->allowCollisions(PART_5_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // KET 8
    const std::string PART_6_NAME = "ket8_link";

    Eigen::Isometry3d ket8_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp facing straight down, X-axis facing (180 deg rotation around X axis)
    ket8_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    ket8_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped ket8_place_pose;
    ket8_place_pose.header.frame_id = "taskboard_link";
    ket8_place_pose.pose.position.x = 0.11755;   // 117.55 mm -> 0.11755 m
    ket8_place_pose.pose.position.y = 0.111824;  // 111.824 mm -> 0.111824 m
    ket8_place_pose.pose.position.z = 0.191;     // Taskboard height

    // Top-down orientation, X axis facing
    ket8_place_pose.pose.orientation.x = 1.0;
    ket8_place_pose.pose.orientation.y = 0.0;
    ket8_place_pose.pose.orientation.z = 0.0;
    ket8_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* ket8_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_6_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_6_NAME, ket8_grasp_tf, solvers, current_state_ptr_, &ket8_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_6_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_6_NAME, ket8_place_pose, solvers, ket8_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_6_NAME);
      stage->allowCollisions(PART_6_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // KET 12
    const std::string PART_7_NAME = "ket12_link";

    Eigen::Isometry3d ket12_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp facing straight down, X-axis facing (180 deg rotation around X axis)
    ket12_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    ket12_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped ket12_place_pose;
    ket12_place_pose.header.frame_id = "taskboard_link";
    ket12_place_pose.pose.position.x = -0.0414496;  // -41.4496 mm -> -0.0414496 m
    ket12_place_pose.pose.position.y = 0.186824;    // 186.824 mm -> 0.186824 m
    ket12_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Top-down orientation, X axis facing
    ket12_place_pose.pose.orientation.x = 1.0;
    ket12_place_pose.pose.orientation.y = 0.0;
    ket12_place_pose.pose.orientation.z = 0.0;
    ket12_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* ket12_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_7_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_7_NAME, ket12_grasp_tf, solvers, current_state_ptr_, &ket12_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_7_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_7_NAME, ket12_place_pose, solvers, ket12_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_7_NAME);
      stage->allowCollisions(PART_7_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // KET 16
    const std::string PART_8_NAME = "ket16_link";

    Eigen::Isometry3d ket16_grasp_tf = Eigen::Isometry3d::Identity();

    // Pick sideways, X-axis facing (90 deg rotation around Y axis)
    // This turns the gripper horizontally instead of pointing straight down
    ket16_grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    ket16_grasp_tf.translation().z() = 0.1042;  // Grasp offset (along tool Z-axis)

    geometry_msgs::msg::PoseStamped ket16_place_pose;
    ket16_place_pose.header.frame_id = "taskboard_link";
    ket16_place_pose.pose.position.x = -0.0414406;  // -41.4406 mm -> -0.0414406 m
    ket16_place_pose.pose.position.y = 0.0388242;   // 38.8242 mm -> 0.0388242 m
    ket16_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Standing, Y-axis facing
    // 90 degree rotation around the X-axis to stand the part up.
    // If it ends up facing X instead of Y in reality, swap to (x=0, y=0.707, z=0, w=0.707)
    ket16_place_pose.pose.orientation.x = 0.7071068;
    ket16_place_pose.pose.orientation.y = 0.0;
    ket16_place_pose.pose.orientation.z = 0.0;
    ket16_place_pose.pose.orientation.w = 0.7071068;

    mtc::Stage* ket16_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_8_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_8_NAME, ket16_grasp_tf, solvers, current_state_ptr_, &ket16_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_8_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_8_NAME, ket16_place_pose, solvers, ket16_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_8_NAME);
      stage->allowCollisions(PART_8_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // RCOG
    // 10. RCOG4_50
    const std::string PART_9_NAME = "rcog4_50_link";

    Eigen::Isometry3d rcog4_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp, X-axis facing (180 deg rotation around X axis)
    rcog4_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    rcog4_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped rcog4_place_pose;
    rcog4_place_pose.header.frame_id = "taskboard_link";
    rcog4_place_pose.pose.position.x = 0.19255;    // 192.55 mm -> 0.19255 m
    rcog4_place_pose.pose.position.y = 0.0368242;  // 36.8242 mm -> 0.0368242 m
    rcog4_place_pose.pose.position.z = 0.191;      // Taskboard height

    // Top-down, X-axis facing rotated by -90 degrees around Z axis
    rcog4_place_pose.pose.orientation.x = 0.7071068;
    rcog4_place_pose.pose.orientation.y = -0.7071068;
    rcog4_place_pose.pose.orientation.z = 0.0;
    rcog4_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* rcog4_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_9_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_9_NAME, rcog4_grasp_tf, solvers, current_state_ptr_, &rcog4_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_9_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_9_NAME, rcog4_place_pose, solvers, rcog4_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_9_NAME);
      stage->allowCollisions(PART_9_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 11. RCOG8_50
    const std::string PART_10_NAME = "rcog8_50_link";

    Eigen::Isometry3d rcog8_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways grasp (90 deg rotation around Y axis)
    rcog8_grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    rcog8_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped rcog8_place_pose;
    rcog8_place_pose.header.frame_id = "taskboard_link";
    rcog8_place_pose.pose.position.x = -0.11645;  // -116.45 mm -> -0.11645 m
    rcog8_place_pose.pose.position.y = 0.036842;  // 36.842 mm -> 0.036842 m
    rcog8_place_pose.pose.position.z = 0.191;     // Taskboard height

    // Standing pose (90 deg pitch rotation around X axis)
    rcog8_place_pose.pose.orientation.x = 0.7071068;
    rcog8_place_pose.pose.orientation.y = 0.0;
    rcog8_place_pose.pose.orientation.z = 0.0;
    rcog8_place_pose.pose.orientation.w = 0.7071068;

    mtc::Stage* rcog8_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_10_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_10_NAME, rcog8_grasp_tf, solvers, current_state_ptr_, &rcog8_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_10_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_10_NAME, rcog8_place_pose, solvers, rcog8_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_10_NAME);
      stage->allowCollisions(PART_10_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 12. RCOG12_50
    const std::string PART_11_NAME = "rcog12_50_link";

    Eigen::Isometry3d rcog12_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways grasp (90 deg rotation around Y axis)
    rcog12_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped rcog12_place_pose;
    rcog12_place_pose.header.frame_id = "taskboard_link";
    rcog12_place_pose.pose.position.x = -0.041996;  // -41.996 mm -> -0.041996 m
    rcog12_place_pose.pose.position.y = -0.122176;  // -122.176 mm -> -0.122176 m
    rcog12_place_pose.pose.position.z = 0.191;      // Taskboard height

    // Standing pose (90 deg pitch rotation around X axis)
    rcog12_place_pose.pose.orientation.x = 0.7071068;
    rcog12_place_pose.pose.orientation.y = 0.0;
    rcog12_place_pose.pose.orientation.z = 0.0;
    rcog12_place_pose.pose.orientation.w = 0.7071068;

    mtc::Stage* rcog12_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_11_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_11_NAME, rcog12_grasp_tf, solvers, current_state_ptr_, &rcog12_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_11_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_11_NAME, rcog12_place_pose, solvers, rcog12_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_11_NAME);
      stage->allowCollisions(PART_11_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 13. RCOG16_50
    const std::string PART_12_NAME = "rcog16_50_link";

    Eigen::Isometry3d rcog16_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways grasp (90 deg rotation around Y axis)
    rcog16_grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    rcog16_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped rcog16_place_pose;
    rcog16_place_pose.header.frame_id = "taskboard_link";
    rcog16_place_pose.pose.position.x = 0.11755;    // 117.55 mm -> 0.11755 m
    rcog16_place_pose.pose.position.y = -0.122176;  // -122.176 mm -> -0.122176 m
    rcog16_place_pose.pose.position.z = 0.191;      // Taskboard height

    // Standing pose (90 deg pitch rotation around X axis)
    rcog16_place_pose.pose.orientation.x = 0.7071068;
    rcog16_place_pose.pose.orientation.y = 0.0;
    rcog16_place_pose.pose.orientation.z = 0.0;
    rcog16_place_pose.pose.orientation.w = 0.7071068;

    mtc::Stage* rcog16_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_12_NAME, solvers.sampling));
    task.add(
        createPickContainer(task, PART_12_NAME, rcog16_grasp_tf, solvers, current_state_ptr_, &rcog16_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_12_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_12_NAME, rcog16_place_pose, solvers, rcog16_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_12_NAME);
      stage->allowCollisions(PART_12_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 14. USB Male
    const std::string PART_13_NAME = "usb_male_link";

    Eigen::Isometry3d usb_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp, Y-axis facing (180 deg rotation around Y axis)
    usb_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    usb_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped usb_place_pose;
    usb_place_pose.header.frame_id = "taskboard_link";
    usb_place_pose.pose.position.x = -0.0414496;  // -41.4496 mm -> -0.0414496 m
    usb_place_pose.pose.position.y = 0.111824;    // 111.824 mm -> 0.111824 m
    usb_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Top-down, Y-axis facing rotated by -90 degrees around Z axis
    usb_place_pose.pose.orientation.x = -0.7071068;
    usb_place_pose.pose.orientation.y = 0.7071068;
    usb_place_pose.pose.orientation.z = 0.0;
    usb_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* usb_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_13_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_13_NAME, usb_grasp_tf, solvers, current_state_ptr_, &usb_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_13_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_13_NAME, usb_place_pose, solvers, usb_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_13_NAME);
      stage->allowCollisions(PART_13_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 15. RJ45 Male
    const std::string PART_14_NAME = "rj45_male_link";

    Eigen::Isometry3d rj45_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp, X-axis facing (180 deg rotation around X axis)
    rj45_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    rj45_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped rj45_place_pose;
    rj45_place_pose.header.frame_id = "taskboard_link";
    rj45_place_pose.pose.position.x = 0.0425504;  // 42.5504 mm -> 0.0425504 m
    rj45_place_pose.pose.position.y = 0.0368242;  // 36.8242 mm -> 0.0368242 m
    rj45_place_pose.pose.position.z = 0.191;      // Taskboard height

    // Top-down, X-axis facing rotated by 90 degrees around Z axis
    rj45_place_pose.pose.orientation.x = 0.7071068;
    rj45_place_pose.pose.orientation.y = -0.7071068;
    rj45_place_pose.pose.orientation.z = 0.0;
    rj45_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* rj45_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_14_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_14_NAME, rj45_grasp_tf, solvers, current_state_ptr_, &rj45_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_14_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_14_NAME, rj45_place_pose, solvers, rj45_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_14_NAME);
      stage->allowCollisions(PART_14_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 16. MCON-E10-SP
    const std::string PART_15_NAME = "mcon-e10-sp_link";

    Eigen::Isometry3d mcon_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways grasp, X-axis facing (90 deg rotation around X axis)
    mcon_grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    mcon_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped mcon_place_pose;
    mcon_place_pose.header.frame_id = "taskboard_link";
    mcon_place_pose.pose.position.x = -0.11645;  // -116.45 mm -> -0.11645 m
    mcon_place_pose.pose.position.y = 0.186824;  // 186.824 mm -> 0.186824 m
    mcon_place_pose.pose.position.z = 0.191;     // Taskboard height

    // Top-down, Y-axis facing rotated 180 degrees
    mcon_place_pose.pose.orientation.x = 1.0;
    mcon_place_pose.pose.orientation.y = 0.0;
    mcon_place_pose.pose.orientation.z = 0.0;
    mcon_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* mcon_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_15_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_15_NAME, mcon_grasp_tf, solvers, current_state_ptr_, &mcon_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_15_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_15_NAME, mcon_place_pose, solvers, mcon_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_15_NAME);
      stage->allowCollisions(PART_15_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 17. BNCP-1.5A-K (BNC Male)
    const std::string PART_16_NAME = "bnc_male_link";

    Eigen::Isometry3d bnc_grasp_tf = Eigen::Isometry3d::Identity();
    // Sideways grasp, Y-axis facing (90 deg rotation around Y axis)
    bnc_grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    bnc_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped bnc_place_pose;
    bnc_place_pose.header.frame_id = "taskboard_link";
    bnc_place_pose.pose.position.x = 0.19255;   // 192.55 mm -> 0.19255 m
    bnc_place_pose.pose.position.y = 0.104324;  // 104.324 mm -> 0.104324 m
    bnc_place_pose.pose.position.z = 0.191;     // Taskboard height

    // Top-down symmetrical orientation
    bnc_place_pose.pose.orientation.x = 0.0;
    bnc_place_pose.pose.orientation.y = 1.0;
    bnc_place_pose.pose.orientation.z = 0.0;
    bnc_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* bnc_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_16_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_16_NAME, bnc_grasp_tf, solvers, current_state_ptr_, &bnc_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_16_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_16_NAME, bnc_place_pose, solvers, bnc_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_16_NAME);
      stage->allowCollisions(PART_16_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 18. DSUB Male
    const std::string PART_17_NAME = "dsub_male_link";

    Eigen::Isometry3d dsub_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp, Y-axis facing (180 deg rotation around Y axis)
    dsub_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    dsub_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped dsub_place_pose;
    dsub_place_pose.header.frame_id = "taskboard_link";
    dsub_place_pose.pose.position.x = 0.11755;   // 117.55 mm -> 0.11755 m
    dsub_place_pose.pose.position.y = 0.186824;  // 186.824 mm -> 0.186824 m
    dsub_place_pose.pose.position.z = 0.191;     // Taskboard height

    // Top-down, Y-axis facing rotated by 90 degrees around Z axis
    dsub_place_pose.pose.orientation.x = 0.7071068;
    dsub_place_pose.pose.orientation.y = 0.7071068;
    dsub_place_pose.pose.orientation.z = 0.0;
    dsub_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* dsub_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_17_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_17_NAME, dsub_grasp_tf, solvers, current_state_ptr_, &dsub_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_17_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_17_NAME, dsub_place_pose, solvers, dsub_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_17_NAME);
      stage->allowCollisions(PART_17_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 19. Small Gear
    const std::string PART_18_NAME = "gear_small_link";

    Eigen::Isometry3d gear_small_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp (180 deg rotation around Y axis)
    gear_small_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    gear_small_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped gear_small_place_pose;
    gear_small_place_pose.header.frame_id = "taskboard_link";
    gear_small_place_pose.pose.position.x = 0.16255;     // 162.55 mm -> 0.16255 m
    gear_small_place_pose.pose.position.y = -0.0471758;  // -47.1758 mm -> -0.0471758 m
    gear_small_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Top-down symmetrical orientation
    gear_small_place_pose.pose.orientation.x = 0.0;
    gear_small_place_pose.pose.orientation.y = 1.0;
    gear_small_place_pose.pose.orientation.z = 0.0;
    gear_small_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* gear_small_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_18_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_18_NAME, gear_small_grasp_tf, solvers, current_state_ptr_,
                                 &gear_small_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_18_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_18_NAME, gear_small_place_pose, solvers, gear_small_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_18_NAME);
      stage->allowCollisions(PART_18_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 20. Medium Gear
    const std::string PART_19_NAME = "gear_medium_link";

    Eigen::Isometry3d gear_medium_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp (180 deg rotation around Y axis)
    gear_medium_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    gear_medium_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped gear_medium_place_pose;
    gear_medium_place_pose.header.frame_id = "taskboard_link";
    gear_medium_place_pose.pose.position.x = 0.19255;     // 192.55 mm -> 0.19255 m
    gear_medium_place_pose.pose.position.y = -0.0471758;  // -47.1758 mm -> -0.0471758 m
    gear_medium_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Top-down symmetrical orientation
    gear_medium_place_pose.pose.orientation.x = 0.0;
    gear_medium_place_pose.pose.orientation.y = 1.0;
    gear_medium_place_pose.pose.orientation.z = 0.0;
    gear_medium_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* gear_medium_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_19_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_19_NAME, gear_medium_grasp_tf, solvers, current_state_ptr_,
                                 &gear_medium_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_19_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_19_NAME, gear_medium_place_pose, solvers, gear_medium_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_19_NAME);
      stage->allowCollisions(PART_19_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    // 21. Large Gear
    const std::string PART_20_NAME = "gear_large_link";

    Eigen::Isometry3d gear_large_grasp_tf = Eigen::Isometry3d::Identity();
    // Top-down grasp (180 deg rotation around Y axis)
    gear_large_grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    gear_large_grasp_tf.translation().z() = 0.1042;  // Grasp offset

    geometry_msgs::msg::PoseStamped gear_large_place_pose;
    gear_large_place_pose.header.frame_id = "taskboard_link";
    gear_large_place_pose.pose.position.x = 0.23925;     // 239.25 mm -> 0.23925 m
    gear_large_place_pose.pose.position.y = -0.0471758;  // -47.1758 mm -> -0.0471758 m
    gear_large_place_pose.pose.position.z = 0.191;       // Taskboard height

    // Top-down symmetrical orientation
    gear_large_place_pose.pose.orientation.x = 0.0;
    gear_large_place_pose.pose.orientation.y = 1.0;
    gear_large_place_pose.pose.orientation.z = 0.0;
    gear_large_place_pose.pose.orientation.w = 0.0;

    mtc::Stage* gear_large_attach_stage = nullptr;

    task.add(createConnectStage("connect_to_pick_" + PART_20_NAME, solvers.sampling));
    task.add(createPickContainer(task, PART_20_NAME, gear_large_grasp_tf, solvers, current_state_ptr_,
                                 &gear_large_attach_stage));

    task.add(createConnectStage("connect_to_place_" + PART_20_NAME, solvers.sampling));
    task.add(createPlaceContainer(task, PART_20_NAME, gear_large_place_pose, solvers, gear_large_attach_stage));

    // Restore environment collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + PART_20_NAME);
      stage->allowCollisions(PART_20_NAME, "taskboard_link", false);
      current_state_ptr_ = stage.get();
      task.add(std::move(stage));
    }

    return task;
  }

  // ---------------------------------------------------------------------------
  // HELPER FUNCTIONS
  // ---------------------------------------------------------------------------

  void setupEnvironment(mtc::Task& task, const Solvers& solvers)
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr_ = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_environment_collisions");
    const auto& robot_model = task.getRobotModel();
    const auto* eef_jmg = robot_model->getJointModelGroup(HAND_GROUP);
    const std::vector<std::string>& robot_links = eef_jmg->getLinkModelNames();

    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> env_objects = psi.getKnownObjectNames();

    for (const auto& obj : env_objects)
    {
      if (obj != "taskboard_link" && obj != "tray_link")
      {
        stage->allowCollisions(obj, robot_links, true);
        stage->allowCollisions(obj, "taskboard_link", true);
        stage->allowCollisions(obj, "tray_link", true);
      }
    }

    // Allow gripper to touch tables
    stage->allowCollisions("tray_link", "gripper_base_link", true);
    stage->allowCollisions("tray_link", "robotiq_85_left_finger_link", true);
    stage->allowCollisions("tray_link", "robotiq_85_right_finger_link", true);
    stage->allowCollisions("taskboard_link", "gripper_base_link", true);
    stage->allowCollisions("taskboard_link", "robotiq_85_left_finger_link", true);
    stage->allowCollisions("taskboard_link", "robotiq_85_right_finger_link", true);

    current_state_ptr_ = stage.get();
    task.add(std::move(stage));

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open_gripper_init", solvers.interpolation);
    stage_open_hand->setGroup(HAND_GROUP);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }

  std::unique_ptr<mtc::stages::Connect> createConnectStage(const std::string& name,
                                                           std::shared_ptr<mtc::solvers::PipelinePlanner> planner)
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        name, mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, planner } });
    stage->setTimeout(2.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    return stage;
  }

  std::unique_ptr<mtc::SerialContainer> createPickContainer(mtc::Task& task, const std::string& part_name,
                                                            const Eigen::Isometry3d& grasp_tf, const Solvers& solvers,
                                                            mtc::Stage* monitored_stage, mtc::Stage** attach_stage_out)
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick_" + part_name);
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
      stage->properties().set("link", EEF_FRAME);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.03, 0.12);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = EEF_FRAME;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_grasp_collisions");
      stage->allowCollisions(part_name, "robotiq_85_left_finger_link", true);
      stage->allowCollisions(part_name, "robotiq_85_right_finger_link", true);
      grasp->insert(std::move(stage));
    }

    // Generate Grasp
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject(part_name);
      stage->setMonitoredStage(monitored_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp_IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_tf, EEF_FRAME);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // Close Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close_gripper", solvers.interpolation);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    // Attach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
      stage->attachObject(part_name, EEF_FRAME);
      *attach_stage_out = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(EEF_FRAME);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = BASE_FRAME;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    return grasp;
  }

  std::unique_ptr<mtc::SerialContainer> createPlaceContainer(mtc::Task& task, const std::string& part_name,
                                                             const geometry_msgs::msg::PoseStamped& target_pose,
                                                             const Solvers& solvers, mtc::Stage* attach_stage,
                                                             const std::string& surface_link = "taskboard_link")
  {
    auto place = std::make_unique<mtc::SerialContainer>("place_" + part_name);
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_place", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.03, 0.1);
      stage->setIKFrame(EEF_FRAME);
      stage->setForwardedProperties({ "target_pose" });

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Allow Assembly Collisions
    // DEBUG code, ignoring most of the links
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_assembly_collisions");
      const std::vector<std::string> gripper_and_nut = { "robotiq_85_left_finger_link",
                                                         "robotiq_85_right_finger_link",
                                                         "gripper_base_link",
                                                         "link_1",
                                                         "link_2",
                                                         "link_3",
                                                         "link_4",
                                                         "link_5",
                                                         "link_6",
                                                         part_name };
      stage->allowCollisions(surface_link, gripper_and_nut, true);
      place->insert(std::move(stage));
    }

    // Generate Place Pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate_pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setObject(part_name);
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place_IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(part_name);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->properties().set("ignore_collisions", true);
      place->insert(std::move(wrapper));
    }

    // Open Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", solvers.interpolation);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // Detach
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
      stage->detachObject(part_name, EEF_FRAME);
      place->insert(std::move(stage));
    }

    // Retract
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(EEF_FRAME);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    return place;
  }

  // PLACE CONTAINER 2: FASTEN (Helical Twist - Rotation + Downward Translation)
  std::unique_ptr<mtc::SerialContainer> createFastenContainer(mtc::Task& task, const std::string& part_name,
                                                              const geometry_msgs::msg::PoseStamped& target_pose,
                                                              const Solvers& solvers, mtc::Stage* attach_stage,
                                                              double twist_angle_rad,
                                                              double thread_pitch = 0.0007)  // Default to standard M4
                                                                                             // pitch (0.7 mm)
  {
    auto place = std::make_unique<mtc::SerialContainer>("fasten_" + part_name);
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 1. Approach Target (Move straight down)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.1);
      stage->setIKFrame(EEF_FRAME);
      stage->setForwardedProperties({ "target_pose" });

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // 2. Allow Assembly Collisions
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collisions");
      const std::vector<std::string> gripper_and_part = { "robotiq_85_left_finger_link",
                                                          "robotiq_85_right_finger_link",
                                                          "gripper_base_link",
                                                          "link_4",
                                                          "link_5",
                                                          "link_6",
                                                          part_name };
      stage->allowCollisions("taskboard_link", gripper_and_part, true);
      place->insert(std::move(stage));
    }

    // 3. Generate Place Pose (Pushes part down to the thread start)
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate_pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setObject(part_name);
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place_IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(part_name);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->properties().set("ignore_collisions", true);
      place->insert(std::move(wrapper));
    }

    // 4. THE SCREW MOTION: Helical twist (rotation + downward translation)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("screw_motion", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setIKFrame(EEF_FRAME);

      // Calculate how far to move along the Z-axis based on the thread pitch
      double turns = twist_angle_rad / (2.0 * M_PI);

      // Assuming +Z on your TCP points OUTWARD from the gripper toward the nut.
      // If +Z points backward into the arm, change this to a negative value!
      double z_forward_distance = thread_pitch * turns;

      geometry_msgs::msg::TwistStamped twist;
      twist.header.frame_id = EEF_FRAME;

      // Combine angular and linear velocities to create a flawless helical path
      twist.twist.angular.z = twist_angle_rad;
      twist.twist.linear.z = z_forward_distance;

      stage->setDirection(twist);
      place->insert(std::move(stage));
    }

    // 5. Open Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", solvers.interpolation);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 6. Detach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
      stage->detachObject(part_name, EEF_FRAME);
      place->insert(std::move(stage));
    }

    // 7. Retract Arm (Move straight up)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(EEF_FRAME);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    return place;
  }

  std::unique_ptr<mtc::SerialContainer> createMeshGearContainer(mtc::Task& task, const std::string& part_name,
                                                                const geometry_msgs::msg::PoseStamped& target_pose,
                                                                const Solvers& solvers, mtc::Stage* attach_stage,
                                                                double mesh_angle_rad)  // E.g., M_PI / 12.0 for a
                                                                                        // 15-degree nudge
  {
    auto place = std::make_unique<mtc::SerialContainer>("mesh_gear_" + part_name);
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 1-3. Approach, Collisions, IK Pose
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.1);
      stage->setIKFrame(EEF_FRAME);
      stage->setForwardedProperties({ "target_pose" });
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collisions");
      const std::vector<std::string> gripper_and_part = { "robotiq_85_left_finger_link",
                                                          "robotiq_85_right_finger_link",
                                                          "gripper_base_link",
                                                          "link_4",
                                                          "link_5",
                                                          "link_6",
                                                          part_name };
      stage->allowCollisions("taskboard_link", gripper_and_part, true);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate_pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setObject(part_name);
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place_IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(part_name);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->properties().set("ignore_collisions", true);
      place->insert(std::move(wrapper));
    }

    // 4. THE ADJUST MOTION (Nudge to interlock gear teeth)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("adjust_teeth", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setIKFrame(EEF_FRAME);

      geometry_msgs::msg::TwistStamped twist;
      twist.header.frame_id = EEF_FRAME;
      twist.twist.angular.z = mesh_angle_rad;

      stage->setDirection(twist);
      place->insert(std::move(stage));
    }

    // 5-7. Open, Detach, Retract
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", solvers.interpolation);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
      stage->detachObject(part_name, EEF_FRAME);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(EEF_FRAME);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    return place;
  }

  std::unique_ptr<mtc::stages::MoveRelative> createTransit(const Solvers& solvers, const Eigen::Vector3d& translation,
                                                           const std::string& frame_id = "world",
                                                           const std::string& name = "transit")
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(name, solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(EEF_FRAME);

    // .norm() automatically calculates the Pythagorean distance of the Eigen vector!
    double target_distance = translation.norm();

    if (target_distance > 1e-6)  // Prevent division by zero
    {
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = frame_id;
      // Normalize the direction
      vec.vector.x = translation.x() / target_distance;
      vec.vector.y = translation.y() / target_distance;
      vec.vector.z = translation.z() / target_distance;

      stage->setDirection(vec);
      stage->setMinMaxDistance(target_distance * 0.99, target_distance);
    }
    else
    {
      stage->setMinMaxDistance(0.0, 0.0);
    }

    return stage;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<TaskConstructor>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->get_node_base_interface());

  auto spin_thread = std::make_unique<std::thread>([&executor]() { executor.spin(); });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}