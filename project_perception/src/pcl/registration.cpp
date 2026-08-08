#include "project_perception/registration.hpp"

Registration::Registration(const rclcpp::NodeOptions& options) : Node("registration", options)
{
}

sensor_msgs::msg::PointCloud2::SharedPtr Registration::transformToWorldFrame(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud, const std::string& target_frame)
{
  // Check
  RCLCPP_INFO(this->get_logger(), "Transforming point cloud to frame: %s", target_frame.c_str());
  if (input_cloud->header.frame_id == target_frame)
  {
    RCLCPP_INFO(this->get_logger(), "Point cloud is already in target frame.");
  }

  // Getting the transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, input_cloud->header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    // return nullptr;
  }

  // Transform the cloud
  Eigen::Affine3d transform_eigen;
  transform_eigen = tf2::transformToEigen(transform_stamped);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_cloud, *temp_cloud);

  pcl::transformPointCloud(*temp_cloud, *transformed_cloud, transform_eigen);

  sensor_msgs::msg::PointCloud2::SharedPtr output_cloud(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud, *output_cloud);
  output_cloud->header.frame_id = target_frame;

  // Log
  RCLCPP_INFO(this->get_logger(), "Succesfully transformed point cloud to frame: %s", target_frame.c_str());

  return output_cloud;
}
sensor_msgs::msg::PointCloud2::SharedPtr
Registration::cropPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                             const std::vector<double>& min_bound, const std::vector<double>& max_bound)
{
}

void Registration::showCloudsLeft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source)
{
  pcl_vis_->removePointCloud("view_port1_target");
  pcl_vis_->removePointCloud("view_port1_source");

  PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_source, 255, 0, 0);
  pcl_vis_->addPointCloud(cloud_target, tgt_h, "view_port1_target", view_port1);
  pcl_vis_->addPointCloud(cloud_source, src_h, "view_port1_source", view_port1);

  PCL_INFO("Press q to begin registration. \n");
  pcl_vis_->spin();
}

void Registration::showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target,
                                   const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source)
{
  pcl_vis_->removePointCloud("source");
  pcl_vis_->removePointCloud("target");

  PointCloudColorHandlerGenericField<pcl::PointNormal> tgt_color_handler(cloud_target, "curvature");
  if (!tgt_color_handler.isCapable())
  {
    PCL_WARN("Cannot create curvature color handler!\n");
  }

  PointCloudColorHandlerGenericField<pcl::PointNormal> src_color_handler(cloud_source, "curvature");
  if (!src_color_handler.isCapable())
  {
    PCL_WARN("Cannot create curvature color handler!\n");
  }

  pcl_vis_->addPointCloud(cloud_target, tgt_color_handler, "target", view_port2);
  pcl_vis_->addPointCloud(cloud_source, src_color_handler, "source", view_port2);

  pcl_vis_->spinOnce();
}

void Registration::loadData(int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD>>& models)
{
  std::string extension(".pcd");

  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string(argv[i]);
    if (fname.size() <= extension.size())
      continue;

    std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int))tolower);

    if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
    {
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile(argv[i], *m.cloud);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

      models.push_back(m);
    }
  }
}

void Registration::pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f& final_transform,
                             bool downsample)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  if (downsample)
  {
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloud_src);
    grid.filter(*src);

    grid.setInputCloud(cloud_tgt);
    grid.filter(*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // Compute surface normals and curvature
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x,
  // y, and z
  float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  pcl_vis_->removePointCloud("source");
  pcl_vis_->removePointCloud("target");

  PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tgt_h(output, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_h(cloud_src, 255, 0, 0);
  pcl_vis_->addPointCloud(output, cloud_tgt_h, "target", view_port2);
  pcl_vis_->addPointCloud(cloud_src, cloud_src_h, "source", view_port2);

  PCL_INFO("Press q to continue the registration.\n");
  pcl_vis_->spin();

  pcl_vis_->removePointCloud("source");
  pcl_vis_->removePointCloud("target");

  // add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
}