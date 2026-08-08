#ifndef PERCEPTION_SERVER_H
#define PERCEPTION_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
/**
 * @brief
 */
class PerceptionServer : public rclcpp::Node
{
public:
  PerceptionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Ignore the param for now
  void handleService(const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr req,
                     moveit_msgs::srv::GetPlanningScene::Response::SharedPtr res);
  void handlePointCloudService(const sensor_msgs::msg::PointCloud2::SharedPtr req,
                               const moveit_msgs::msg::PlanningScene::SharedPtr res);
  void handlePlanningSceneService(const moveit_msgs::msg::PlanningScene::SharedPtr req,
                                  const moveit_msgs::msg::PlanningScene::SharedPtr res);
  void createPointCloudService();
  void createPlanningSceneService();
  void createService();
  void createSubscriber();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_image_;
  std::string point_cloud_topic_;
  std::string rgb_image_topic_;

  void declareParameter();
  // PlanningScene
  // Async
  void createAsyncPlanningScene();
  void updateAsyncPlanningScene(moveit_msgs::msg::PlanningScene obj);
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;

  // Sync
  void createSyncPlanningScene();
  void updateSyncPlanningScene(moveit_msgs::srv::ApplyPlanningScene obj);
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_client_;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Debug
};

#endif