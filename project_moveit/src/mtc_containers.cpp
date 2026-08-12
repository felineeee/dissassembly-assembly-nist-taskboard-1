#include "project_moveit/mtc_containers.hpp"

namespace irb120_assembly
{

std::unique_ptr<mtc::stages::Connect> createConnectStage(const std::string& name, const std::string& arm_group,
                                                         std::shared_ptr<mtc::solvers::PipelinePlanner> planner)
{
  auto stage =
      std::make_unique<mtc::stages::Connect>(name, mtc::stages::Connect::GroupPlannerVector{ { arm_group, planner } });
  stage->setTimeout(2.0);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  return stage;
}

std::unique_ptr<mtc::SerialContainer> createPickContainer(mtc::Task& task, const RobotConfig& config,
                                                          const std::string& part_name,
                                                          const Eigen::Isometry3d& grasp_tf, const Solvers& solvers,
                                                          mtc::Stage* monitored_stage, mtc::Stage** attach_stage_out)
{
  auto grasp = std::make_unique<mtc::SerialContainer>("pick_" + part_name);
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_grasp_collisions");
    stage->allowCollisions(part_name, "robotiq_85_left_finger_link", true);
    stage->allowCollisions(part_name, "robotiq_85_right_finger_link", true);

    // DEBUG
    const std::vector<std::string> arm_debug_links = { "link_4", "link_5", "link_6" };
    stage->allowCollisions("taskboard_link", arm_debug_links, true);

    grasp->insert(std::move(stage));
  }

  // Approach
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
    stage->properties().set("link", config.eef_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.03, 0.12);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = config.eef_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
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
    wrapper->setIKFrame(grasp_tf, config.eef_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
  }

  // Close Gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close_gripper", solvers.interpolation);
    stage->setGroup(config.hand_group);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  // Attach Object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
    stage->attachObject(part_name, config.eef_frame);
    *attach_stage_out = stage.get();
    grasp->insert(std::move(stage));
  }

  // Lift Object
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.2);
    stage->setIKFrame(config.eef_frame);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = config.base_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  return grasp;
}

std::unique_ptr<mtc::SerialContainer> createPlaceContainer(mtc::Task& task, const RobotConfig& config,
                                                           const std::string& part_name,
                                                           const geometry_msgs::msg::PoseStamped& target_pose,
                                                           const Solvers& solvers, mtc::Stage* attach_stage,
                                                           const std::string& surface_link)
{
  auto place = std::make_unique<mtc::SerialContainer>("place_" + part_name);
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  // Allow Assembly Collisions
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_assembly_collisions");
    const std::vector<std::string> gripper_and_nut = { "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
                                                       //  "robotiq_85_right_finger_tip_link",
                                                       //  "robotiq_85_left_finger_tip_link",
                                                       "gripper_base_link", part_name };
    stage->allowCollisions(surface_link, gripper_and_nut, true);
    place->insert(std::move(stage));
  }

  // Approach
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_place", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.001, 0.05);
    stage->setIKFrame(config.eef_frame);
    stage->setForwardedProperties({ "target_pose" });

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
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
    stage->setGroup(config.hand_group);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  // Detach
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
    stage->detachObject(part_name, config.eef_frame);
    place->insert(std::move(stage));
  }

  // Retract
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.2);
    stage->setIKFrame(config.eef_frame);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  return place;
}

std::unique_ptr<mtc::SerialContainer> createFastenContainer(mtc::Task& task, const RobotConfig& config,
                                                            const std::string& part_name,
                                                            const geometry_msgs::msg::PoseStamped& target_pose,
                                                            const Solvers& solvers, mtc::Stage* attach_stage,
                                                            double twist_angle_rad, double thread_pitch,
                                                            const std::string& surface_link)
{
  auto place = std::make_unique<mtc::SerialContainer>("fasten_" + part_name);
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  // DEBUG Swap steps
  // 2. Allow Collisions
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collisions");

    stage->allowCollisions(part_name, surface_link, true);
    stage->allowCollisions(surface_link, part_name, true);

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

  // 1. Approach Target
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.01, 0.1);
    stage->setIKFrame(config.eef_frame);
    stage->setForwardedProperties({ "target_pose" });

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  // 3. Generate Place Pose
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

  // 4. Helical Screw Motion
  // 4. HELICAL SCREW MOTION (4 steps of 90 deg = 360 deg total)
  {
    double turns = twist_angle_rad / (2.0 * M_PI);
    double total_z_forward = thread_pitch * turns;

    double max_step_angle = M_PI / 2.0;  // 90 degrees
    int num_steps = std::ceil(std::abs(twist_angle_rad) / max_step_angle);
    if (num_steps < 1)
      num_steps = 1;

    double step_angle = twist_angle_rad / num_steps;
    double step_z = total_z_forward / num_steps;

    for (int i = 0; i < num_steps; ++i)
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("screw_step_" + std::to_string(i + 1), solvers.cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setIKFrame(config.eef_frame);

      geometry_msgs::msg::TwistStamped twist;
      twist.header.frame_id = config.eef_frame;
      twist.twist.angular.z = step_angle;
      twist.twist.linear.z = step_z;

      stage->setDirection(twist);
      place->insert(std::move(stage));
    }
  }

  // 5. Open Gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", solvers.interpolation);
    stage->setGroup(config.hand_group);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  // 6. Detach Object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
    stage->detachObject(part_name, config.eef_frame);
    place->insert(std::move(stage));
  }

  // 7. Retract Arm
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.01, 0.1);
    stage->setIKFrame(config.eef_frame);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  return place;
}

std::unique_ptr<mtc::SerialContainer> createMeshGearContainer(mtc::Task& task, const RobotConfig& config,
                                                              const std::string& part_name,
                                                              const geometry_msgs::msg::PoseStamped& target_pose,
                                                              const Solvers& solvers, mtc::Stage* attach_stage,
                                                              double mesh_angle_rad)
{
  auto place = std::make_unique<mtc::SerialContainer>("mesh_gear_" + part_name);
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  // 1. Approach
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.01, 0.1);
    stage->setIKFrame(config.eef_frame);
    stage->setForwardedProperties({ "target_pose" });

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  // 2. Allow Collisions
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

  // 3. Generate Place Pose
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

  // 4. Adjust Teeth Motion
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("adjust_teeth", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(config.eef_frame);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = config.eef_frame;
    twist.twist.angular.z = mesh_angle_rad;

    stage->setDirection(twist);
    place->insert(std::move(stage));
  }

  // 5. Open Gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", solvers.interpolation);
    stage->setGroup(config.hand_group);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  // 6. Detach
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
    stage->detachObject(part_name, config.eef_frame);
    place->insert(std::move(stage));
  }

  // 7. Retract
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", solvers.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.2);
    stage->setIKFrame(config.eef_frame);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  return place;
}

std::unique_ptr<mtc::stages::MoveRelative> createTransit(const RobotConfig& config, const Solvers& solvers,
                                                         const Eigen::Vector3d& translation,
                                                         const std::string& frame_id, const std::string& name)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(name, solvers.cartesian);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setIKFrame(config.eef_frame);

  double target_distance = translation.norm();

  if (target_distance > 1e-6)
  {
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = frame_id;
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

void restoreCollision(mtc::Task& task, const std::string& part_name, const std::string& target_link,
                      mtc::Stage*& current_state_ptr)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions_" + part_name);
  stage->allowCollisions(part_name, target_link, false);
  current_state_ptr = stage.get();
  task.add(std::move(stage));
}

}  // namespace irb120_assembly