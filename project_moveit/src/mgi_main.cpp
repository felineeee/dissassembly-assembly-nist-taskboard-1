#include "irb1200_moveit/planning_mgi.hpp"
// Member function - [this] valid
// mgi_main.cpp
void setupNutPickTask(PlanningMGI& planning, const std::string& nut_id)
{
  planning.addStage("approach_nut", [planning = std::ref(planning), nut_id]() -> bool {
    auto pose = planning.get().loadPoseFromYAML(nut_id, "taskboard_v2.yaml");
    pose.position.z += 0.10;
    return planning.get().planExecutePose(pose);
  });
  planning.addStage("pre_grasp", [planning = std::ref(planning), nut_id]() -> bool {
    auto pose = planning.get().loadPoseFromYAML(nut_id, "taskboard_v2.yaml");
    pose.position.z += 0.02;
    return planning.get().planExecutePose(pose);
  });
  planning.addStage("open_grasp", [planning = std::ref(planning), nut_id]() -> bool {
    auto pose = planning.get().loadPoseFromYAML(nut_id, "taskboard_v2.yaml");
    // planning.setGripperPosition()
    pose.position.z += 0.10;
    return planning.get().planExecutePose(pose);
  });
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PlanningMGI>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  node->initializeNode();
  //   node->planAndExecute();

  spinner.join();
  rclcpp::shutdown();

  return 0;
}