#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "project_moveit/task_constructor_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<irb120_assembly::TaskConstructor>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->get_node_base_interface());

  auto spin_thread = std::make_unique<std::thread>([&executor]() { executor.spin(); });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}