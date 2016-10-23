#include "common.h"

void voxelFitler(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, float td)
{
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(td, td, td);
    voxel.filter(*cloud_out);
}

void statisticFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, int meank, double devMulThresh)
{
    pcl::StatisticalOutlierRemoval<PointT> sorSta;
    sorSta.setInputCloud( cloud_in);
    sorSta.setMeanK (meank);
    sorSta.setStddevMulThresh (devMulThresh);
    sorSta.filter (*cloud_out);
}

void radiusFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, double r, int min_neighbors)
{
    pcl::RadiusOutlierRemoval<PointT> radius;
    radius.setInputCloud(cloud_in);
    radius.setRadiusSearch(r);
    radius.setMinNeighborsInRadius(min_neighbors);
    radius.filter(*cloud_out);
}
