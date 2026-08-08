#include "project_perception/perception_server.hpp"

PerceptionServer::PerceptionServer(const rclcpp::NodeOptions& options) : Node("perception_server", options)
{
  declareParameter();
  createSubscriber();
  createSyncPlanningScene();
  createAsyncPlanningScene();
};

// Planning Scene
void createAsyncPlanningScene()
{
}

// Param
void PerceptionServer::declareParameter()
{
  point_cloud_topic_ = this->declare_parameter<std::string>("point_cloud_topic", "/camera/depth_registered/points");
  rgb_image_topic_ = this->declare_parameter<std::string>("rgb_image_topic", "/camera/color/image_raw");
}
// Service
void PerceptionServer::handleService(const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr req,
                                     moveit_msgs::srv::GetPlanningScene::Response::SharedPtr res)
{
  // Check if its empty
  if (!latest_point_cloud_ || latest_point_cloud_->data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Latest point cloud is empty.");
    return;
  }
  if (!latest_rgb_image_ || latest_rgb_image_->data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Latest RGB image is empty.");
    return;
  }
  // Filter
  // Segmentation
  // (Registration) Transform
  // Reconstruction
}

void PerceptionServer::createService()
{
}

void PerceptionServer::createSubscriber()
{
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pointCloudCallback(msg); });
  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      rgb_image_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) { rgbImageCallback(msg); });
}
void PerceptionServer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg != nullptr && !msg->data.empty())
  {
    latest_point_cloud_ = msg;
  }
}
void PerceptionServer::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg != nullptr && !msg->data.empty())
  {
    latest_rgb_image_ = msg;
  }
}

// Planning Scene
void PerceptionServer::createAsyncPlanningScene()
{
  planning_scene_publisher_ =
      this->create_publisher<moveit_msgs::msg::PlanningScene>("async_planning_scene_client", 10);
}
void PerceptionServer::createSyncPlanningScene()
{
  planning_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("sync_planning_scene_client");
}
void PerceptionServer::updateAsyncPlanningScene(moveit_msgs::msg::PlanningScene obj)
{
  planning_scene_publisher_->publish(obj);
}
void PerceptionServer::updateSyncPlanningScene(moveit_msgs::srv::ApplyPlanningScene obj)
{
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  // request->scene.world.collision_objects.push_back(obj);
  // request->scene.is_diff = true;
  auto future = planning_scene_client_->async_send_request(request);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dummy_node");
  RCLCPP_INFO(node->get_logger(), "Dummy node has started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}