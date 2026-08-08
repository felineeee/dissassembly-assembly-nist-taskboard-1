#ifndef EXTRACTION_FILTER_H
#define EXTRACTION_FILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

template <typename PointT>
class ExtractionFilter
{
  typename pcl::PointCloud<PointT>::Ptr cloud_;

  void conditionalFilter();
  void extractIndices();
};

#endif