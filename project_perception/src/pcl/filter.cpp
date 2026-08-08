#include "project_perception/filter.hpp"

/**
 *
 * @brief This code will be used for filtering noise. Working on the data types of PCL to PCL. Conversion from ROS to
 * PCL vice versa will be handled by service
 * @note should return ptr if to adjust the performance
 */
sensor_msgs::msg::PointCloud2 Filter::passtroughFilter(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                                       double min_bound, double max_bound)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud, *filtered_cloud);

  pcl::PassThrough<pcl::PointXYZ> passtrough_filter;
  passtrough_filter.setInputCloud(filtered_cloud);
  passtrough_filter.setFilterFieldName("z");
  passtrough_filter.setFilterLimits(min_bound, max_bound);
  passtrough_filter.filter(*filtered_cloud);

  sensor_msgs::msg::PointCloud2 output_cloud;
  pcl::toROSMsg(*filtered_cloud, output_cloud);
  return output_cloud;
}

sensor_msgs::msg::PointCloud2 Filter::downsampleVoxelGrid(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                                          float lx, float ly, float lz)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_cloud, *filtered_cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(filtered_cloud);
  voxel_grid.setLeafSize(lx, ly, lz);
  voxel_grid.filter(*filtered_cloud);

  sensor_msgs::msg::PointCloud2 output_cloud;
  pcl::toROSMsg(*filtered_cloud, output_cloud);
  return output_cloud;
}

void Filter::parametricModelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, float a, float b, float c,
                                   float d)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = a;
  coefficients->values[1] = b;
  coefficients->values[2] = c;
  coefficients->values[3] = d;

  pcl::ProjectInliers<pcl::PointXYZRGB> project_inliers;
  project_inliers.setModelType(pcl::SACMODEL_PLANE);
  project_inliers.setModelCoefficients(coefficients);
  project_inliers.setInputCloud(input_cloud);
  project_inliers.filter(*input_cloud);
}

void Filter::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int mean_k,
                                       double std_dev_mul)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_dev_mul);
  sor.filter(*input_cloud);
}