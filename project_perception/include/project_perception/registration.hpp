#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

struct PCD
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::string f_name;

  PCD() : cloud(new pcl::PointCloud<pcl::PointXYZ>()) {};
};

class Registration : public rclcpp::Node
{
public:
  Registration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  sensor_msgs::msg::PointCloud2::SharedPtr

  // Transform
  transformToWorldFrame(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud, const std::string& target_frame);

  // Crop PointCLoud
  sensor_msgs::msg::PointCloud2::SharedPtr cropPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                                          const std::vector<double>& min_bound,
                                                          const std::vector<double>& max_bound);

  // ICP Pipeline
  void loadData(int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD>>& models);
  void pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                 Eigen::Matrix4f& final_transform, bool downsample = false);

  // Visualization
  void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source);
  void showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target,
                       const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source);

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  pcl::visualization::PCLVisualizer* pcl_vis_;

  int view_port1, view_port2;
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
  MyPointRepresentation()
  {
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const pcl::PointNormal& p, float* out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

#endif