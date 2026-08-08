#ifndef FILTER_H
#define FILTER_H

#include "rclcpp/rclcpp.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Filter : public rclcpp::Node {
public:
  Filter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  sensor_msgs::msg::PointCloud2
  passtroughFilter(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud,
                   double min_bound, double max_bound);
  sensor_msgs::msg::PointCloud2 downsampleVoxelGrid(
      const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud, float lx,
      float ly, float lz);
  /**
   * @brief Taking  3D Point Cloud and mathematically squash them to geometric
   * shape
   * @note ax + by + cz + d = 0, where a,b,c is normal vector and d is distance
   * of the plane from the origin along that normal vector.
   */
  void
  parametricModelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                        float a, float b, float c, float d);
  void
  statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                            int mean_k, double std_dev_mul);

private:
};
#endif