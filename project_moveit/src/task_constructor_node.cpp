#include "project_moveit/task_constructor_node.hpp"

namespace irb120_assembly
{

TaskConstructor::TaskConstructor(const rclcpp::NodeOptions& options) : Node("task_constructor", options)
{
}

void TaskConstructor::doTask()
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

void TaskConstructor::setupEnvironment(mtc::Task& task)
{
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr_ = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_environment_collisions");
  const auto& robot_model = task.getRobotModel();
  const auto* eef_jmg = robot_model->getJointModelGroup(config_.hand_group);
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

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open_gripper_init", solvers_.interpolation);
  stage_open_hand->setGroup(config_.hand_group);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
}

mtc::Task TaskConstructor::createTask()
{
  mtc::Task task;
  task.stages()->setName("Explicit Assembly Task");
  task.loadRobotModel(shared_from_this());

  task.setProperty("group", config_.arm_group);
  task.setProperty("eef", config_.hand_group);
  task.setProperty("ik_frame", config_.eef_frame);

  // Initialize Solvers
  solvers_.sampling = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  solvers_.interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  solvers_.cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  solvers_.cartesian->setMaxVelocityScalingFactor(0.2);
  solvers_.cartesian->setMaxAccelerationScalingFactor(0.2);
  solvers_.cartesian->setStepSize(0.005);

  // 1. Setup Initial State
  setupEnvironment(task);

  // DEBUG
  // 2. Build Modular Assembly Steps
  addNutTasks(task);
  // addKetTasks(task);
  // addRcogTasks(task);
  // addConnectorTasks(task);
  // addGearTasks(task);

  return task;
}

// -----------------------------------------------------------------------------
// PART BUILDERS
// -----------------------------------------------------------------------------

void TaskConstructor::addNutTasks(mtc::Task& task)
{
  // -- -M4 Nut-- -`
  {
    const std::string name = "m4_nut_link";
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // grasp_tf.translation().z() = 0.1118;  // 0.115 - 0.0032
    // grasp_tf.translation().z() = 0.0032;  // 0.115 - 0.0032
    // grasp_tf.translation().z() = 0.115;  // 0.115 - 0.0032

    // Its 8.5mm, not 0.115m
    // 8.5-3.2 = 5.3mm

    // 0.0426 from jaw
    // 0.144 from base
    // 0.0426-0.0032 (m4 nut height) onrobot-2fg7
    // 0.032 - 0.0032 robotiq 2f-85
    grasp_tf.translation().z() = 0.009;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";

    pose.pose.position.x = 0.0005;
    pose.pose.position.y = -0.075;
    pose.pose.position.z = 0.007;  // taskboard 0.1755,  but this relative

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    double pitch = 0.0007;

    // double twist = (0.007 / pitch) * (2.0 * M_PI); // Original calculation formula
    double max_turns = 1.0;
    double twist = max_turns * (2.0 * M_PI);

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_fasten_" + name, config_.arm_group, solvers_.sampling));
    task.add(createFastenContainer(task, config_, name, pose, solvers_, attach_stage, twist, pitch));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }

  // -- -M8 Nut-- -
  {
    const std::string name = "m8_nut_link";
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // 0.0068
    grasp_tf.translation().z() = 0.009;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    // I forgot this number relates to what. maybe thread length - board height

    pose.pose.position.x = 0.150;
    pose.pose.position.y = -0.150;
    pose.pose.position.z = 0.016;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    double pitch = 0.00125;
    double max_turns = 1.0;
    double twist = max_turns * (2.0 * M_PI);

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_fasten_" + name, config_.arm_group, solvers_.sampling));
    task.add(createFastenContainer(task, config_, name, pose, solvers_, attach_stage, twist, pitch));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }

  // -- -M12 Nut-- -
  {
    const std::string name = "m12_nut_link";
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // 0.0108
    // grasp_tf.translation().z() = 0.0318;
    grasp_tf.translation().z() = 0.009;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    pose.pose.position.x = -0.0745;
    pose.pose.position.y = 0.153;
    pose.pose.position.z = 0.011;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    double pitch = 0.00175;
    double max_turns = 1.0;
    double twist = max_turns * (2.0 * M_PI);

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_fasten_" + name, config_.arm_group, solvers_.sampling));
    task.add(createFastenContainer(task, config_, name, pose, solvers_, attach_stage, twist, pitch));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }

  // --- M16 Nut ---
  // UNSOLVED
  // {
  //   const std::string name = "m16_nut_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   grasp_tf.linear() = (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
  //                        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()))
  //                           .toRotationMatrix();

  //   // 0.0148
  //   grasp_tf.translation().z() = 0.0278;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";

  //   pose.pose.position.x = -0.150;
  //   pose.pose.position.y = -0.145;
  //   pose.pose.position.z = 0.009;

  //   pose.pose.orientation.x = 0.7071;
  //   pose.pose.orientation.y = 0.7071;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;

  //   double pitch = 0.002;
  //   double max_turns = 1.0;
  //   double twist = max_turns * (2.0 * M_PI);

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_fasten_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createFastenContainer(task, config_, name, pose, solvers_, attach_stage, twist, pitch));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return_home", solvers_.sampling);
    stage->setGroup(config_.arm_group);
    stage->setGoal("home");
    task.add(std::move(stage));
  }
}

void TaskConstructor::addKetTasks(mtc::Task& task)
{
  /*
   * So, grasp_tf is a relative transformation matrix that defines where the End-Effector (TCP) needs to go relative to
   * the Object frame (ket4_link). When you multiply rotation matrices from left to right, You are applying two
   * sequential rotations in the local reference frame.
   */

  // KET 4
  // UNSOLVED
  /*
   * - Pick->Slide-in is impossible due to the width of the base
   * - Pick->Stage->Pick->Slide-in is still error on cartesian_path
   */
  // {
  //   const std::string name = "ket4_link";

  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   Eigen::Matrix3d R_gripper_down = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  //   Eigen::Matrix3d R_mesh_fix = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   double edge_offset_x = 0.0;
  //   double edge_offset_y = -0.024;
  //   double depth_offset_z = 0.0;
  //   grasp_tf.translation() = Eigen::Vector3d(edge_offset_x, edge_offset_y, depth_offset_z);

  //   grasp_tf.linear() = R_gripper_down * R_mesh_fix;
  //   // Robotic 2f length: 0.0038
  //   grasp_tf.translation().z() = 0.0034;  //-0.0004 KET4

  //   geometry_msgs::msg::PoseStamped staging_pose;
  //   staging_pose.header.frame_id = "tray_link";

  //   staging_pose.pose.position.x = 0.0;
  //   staging_pose.pose.position.y = -0.160;
  //   staging_pose.pose.position.z = 0.035;
  //   Eigen::Matrix3d R_to_upright = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   Eigen::Matrix3d R_standing = R_to_upright * R_mesh_fix;
  //   Eigen::Quaterniond q_standing(R_standing);
  //   q_standing.normalize();

  //   staging_pose.pose.orientation.x = q_standing.x();
  //   staging_pose.pose.orientation.y = q_standing.y();
  //   staging_pose.pose.orientation.z = q_standing.z();
  //   staging_pose.pose.orientation.w = q_standing.w();

  //   // Attach
  //   geometry_msgs::msg::PoseStamped attach_pose;
  //   attach_pose.header.frame_id = "taskboard_link";
  //   attach_pose.pose.position.x = 0.00055;
  //   attach_pose.pose.position.y = 0.0052;
  //   attach_pose.pose.position.z = 0.003;  // 300mm

  //   mtc::Stage* attach_stage = nullptr;

  //   // Staging
  //   // task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   // task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   // task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   // task.add(createPlaceContainer(task, config_, name, staging_pose, solvers_, attach_stage));

  //   // Place
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, attach_pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // -- -KET 8 -- -
  // UNSOLVED
  // {
  //   const std::string name = "ket8_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   grasp_tf.translation().z() = 0.1042;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = 0.11755;
  //   pose.pose.position.y = 0.111824;
  //   pose.pose.position.z = 0.191;
  //   pose.pose.orientation.x = 1.0;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // --- KET 12 ---
  // UNSOLVED
  // {
  //   const std::string name = "ket12_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   grasp_tf.translation().z() = 0.1042;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = -0.0414496;
  //   pose.pose.position.y = 0.186824;
  //   pose.pose.position.z = 0.191;
  //   pose.pose.orientation.x = 1.0;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // --- KET 16 ---
  {
    const std::string name = "ket16_link";
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    // 1. Initial top-down orientation (180 deg pitch around Y)
    Eigen::Matrix3d top_down = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // 2. 90 deg rotation around Z
    Eigen::Matrix3d z_rotation = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 3. Combine them (Post-multiply for local frame rotation)
    grasp_tf.linear() = top_down * z_rotation;

    // 0.032 - 0.004
    grasp_tf.translation().z() = 0.0028;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    pose.pose.position.x = 0.15;
    pose.pose.position.y = 0.005;
    pose.pose.position.z = 0.08;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.7071068;
    pose.pose.orientation.w = 0.7071068;

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return_home", solvers_.sampling);
    stage->setGroup(config_.arm_group);
    stage->setGoal("home");
    task.add(std::move(stage));
  }
}

void TaskConstructor::addRcogTasks(mtc::Task& task)
{
  // --- RCOG 4 ---
  // UNSOLVED
  // {
  //   const std::string name = "rcog4_50_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   grasp_tf.translation().z() = 0.1042;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = 0.19255;
  //   pose.pose.position.y = 0.0368242;
  //   pose.pose.position.z = 0.191;
  //   pose.pose.orientation.x = 0.7071068;
  //   pose.pose.orientation.y = -0.7071068;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // --- RCOG 8, 12, 16 ---
  const std::vector<std::tuple<std::string, double, double>> rcogs = { { "rcog8-50_link", 0.075, 0.15 },
                                                                       { "rcog12-50_link", 0.15, -0.07 },
                                                                       { "rcog16-50_link", -0.075, -0.07 } };
  //
  //   };

  for (const auto& [name, x, y] : rcogs)
  {
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

    grasp_tf.translation().z() = 0.028;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.08;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.7071068;
    pose.pose.orientation.w = 0.7071068;

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }
}

void TaskConstructor::addConnectorTasks(mtc::Task& task)
{
  // USB Male
  // {
  //   const std::string name = "usb_male_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   Eigen::Matrix3d top_down = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  //   Eigen::Matrix3d yaw = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  //   Eigen::Matrix3d z_to_up = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();

  //   grasp_tf.linear() = top_down * yaw * z_to_up;
  //   grasp_tf.translation().z() = 0.034;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = 0.15;
  //   pose.pose.position.y = 0.075;
  //   pose.pose.position.z = 0.050;

  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.7071068;
  //   pose.pose.orientation.w = 0.7071068;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // RJ45 Male
  // Correct pose but,
  // Cartesian Err
  // {
  //   const std::string name = "rj45_male_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();

  //   Eigen::Matrix3d yaw = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  //   Eigen::Matrix3d roll = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  //   grasp_tf.linear() = yaw * roll;

  //   grasp_tf.translation().z() = 0.034;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = -0.15;
  //   pose.pose.position.y = 0.15;
  //   pose.pose.position.z = 0.05;

  //   pose.pose.orientation.x = -0.5;
  //   pose.pose.orientation.y = -0.5;
  //   pose.pose.orientation.z = -0.5;
  //   pose.pose.orientation.w = 0.5;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // MCON-E10-SP
  // Need to be rotated x 180
  // Cant be solved for now
  // {
  //   const std::string name = "mcon-e10-sp_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();

  //   Eigen::Matrix3d rot_z = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  //   Eigen::Matrix3d rot_x = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  //   grasp_tf.linear() = rot_z * rot_x;
  //   grasp_tf.translation().z() = 0.004;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = 0.075;
  //   pose.pose.position.y = 0.0;
  //   pose.pose.position.z = 0.036;

  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.7071068;
  //   pose.pose.orientation.w = 0.7071068;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // BNCP-1.5A-K (BNC Male)
  // Need rotation x180
  // {
  //   const std::string name = "bnc_male_link";
  //   Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
  //   grasp_tf.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  //   grasp_tf.translation().z() = 0.1042;

  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "taskboard_link";
  //   pose.pose.position.x = 0.19255;
  //   pose.pose.position.y = 0.104324;
  //   pose.pose.position.z = 0.191;
  //   pose.pose.orientation.y = 1.0;

  //   mtc::Stage* attach_stage = nullptr;
  //   task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

  //   task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
  //   task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

  //   restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  // }

  // DSUB Male
  {
    const std::string name = "dsub_male_link";
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot_x = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    grasp_tf.linear() = rot_x;
    grasp_tf.translation().z() = 0.034;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    pose.pose.position.x = -0.075;
    pose.pose.position.y = 0.005;
    pose.pose.position.z = 0.036;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1;

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }
}

void TaskConstructor::addGearTasks(mtc::Task& task)
{
  const std::vector<std::tuple<std::string, double>> gears = { { "gear_small_link", 0.16255 },
                                                               { "gear_medium_link", 0.19255 },
                                                               { "gear_large_link", 0.23925 } };

  for (const auto& [name, x] : gears)
  {
    Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
    grasp_tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    grasp_tf.translation().z() = 0.1042;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "taskboard_link";
    pose.pose.position.x = x;
    pose.pose.position.y = -0.0471758;
    pose.pose.position.z = 0.191;
    pose.pose.orientation.y = 1.0;

    mtc::Stage* attach_stage = nullptr;
    task.add(createConnectStage("connect_to_pick_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPickContainer(task, config_, name, grasp_tf, solvers_, current_state_ptr_, &attach_stage));

    task.add(createConnectStage("connect_to_place_" + name, config_.arm_group, solvers_.sampling));
    task.add(createPlaceContainer(task, config_, name, pose, solvers_, attach_stage));

    restoreCollision(task, name, "taskboard_link", current_state_ptr_);
  }
}

}  // namespace irb120_assembly