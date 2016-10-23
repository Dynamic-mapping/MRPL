#pragma once
#include "common.h"

void voxelFitler(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, float td);

void statisticFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, int meank, double devMulThresh);

void radiusFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out, double r, int min_neighbors);
