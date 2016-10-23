#ifndef COMMON_H
#define COMMON_H

// STL
#include <iostream>
//#include <random>

// CUDA STL
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>

// PCL COMMON
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// PCL FILTER
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// PCL ICP
#include <pcl/registration/icp.h>

// PCL VISUAL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "timer.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> CustomColor;
typedef pcl::registration::TransformationEstimationSVD<PointT, PointT> TransRotation;

#endif // COMMON_H
