#ifndef _MRPF_NODE_
#define _MRPF_NODE_

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


namespace mrpf {

typedef std::vector<int> STDvector;
typedef Eigen::VectorXf VectorXf;
typedef pcl::ModelCoefficients ModelCoefficients;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXY PointPlane;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<int> PointCloudIndex;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;


typedef pcl::VoxelGrid<PointType> VoxeGrid;
typedef pcl::SampleConsensusModelParallelPlane<PointType> ParallelPlane;
typedef pcl::SampleConsensusModelParallelPlane<PointType>::Ptr ParallelPlanePtr;

typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointType> CustomColor;
typedef pcl::registration::TransformationEstimationSVD<PointType,PointType> TransRotation;

ModelCoefficients Trans(
        PointCloudPtr &cloud,
        PointCloudPtr &Tcloud,
        PointCloudPtr &ground,
        PointCloudPtr &Tground,
        VectorXf coefficients)
{
    float trans_data[3];
    float base_trans = sqrt(pow(coefficients[0],2)+pow(coefficients[1],2)+pow(coefficients[2],2))/(-coefficients[3]);
    trans_data[0] = coefficients[0]/base_trans;
    trans_data[1] = coefficients[1]/base_trans;
    trans_data[2] = coefficients[2]/base_trans;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -trans_data[0], -trans_data[1], -trans_data[2];

    pcl::transformPointCloud(*cloud, *Tcloud, transform);
    pcl::transformPointCloud(*ground, *Tground, transform);

    PointType A1, A2, A3, B1, B2, B3;
    A1.x = 0;
    A1.y = 0;
    A1.z = 0;
    A2.x = 1;
    A2.y = 0;
    A2.z = -(coefficients[0])/coefficients[2];
    A3.x = 0;
    A3.y = 1;
    A3.z = -(coefficients[1])/coefficients[2];

    B1.x = 0;
    B1.y = 0;
    B1.z = 0;
    B2.x = 1;
    B2.y = 0;
    B2.z = 0;
    B3.x = 0;
    B3.y = 1;
    B3.z = 0;

    pcl::PointCloud<PointType> pc1, pc2;
    pc1.points.push_back(A1);
    pc1.points.push_back(A2);
    pc1.points.push_back(A3);
    pc2.points.push_back(B1);
    pc2.points.push_back(B2);
    pc2.points.push_back(B3);
    Eigen::Matrix4f m=Eigen::Matrix4f::Identity();
    TransRotation rotation (new TransRotation);
    rotation.estimateRigidTransformation(pc1,pc2,m);
    std::cout << m << std::endl;
    pcl::transformPointCloud(*Tcloud, *Tcloud, m);
    pcl::transformPointCloud(*Tground, *Tground, m);

    ModelCoefficients coeff;
    coeff.values.push_back(coefficients[0]);
    coeff.values.push_back(coefficients[1]);
    coeff.values.push_back(coefficients[2]);
    coeff.values.push_back(0.0);
    return coeff;
}



void
thresholdDepth (
        PointCloudPtr & input,
        PointCloudPtr & out,
        float min_depth,
        float max_depth)
    {
      pcl::PassThrough<PointType> pass_through;
      pass_through.setInputCloud (input);
      pass_through.setFilterFieldName ("y");
      pass_through.setFilterLimits (min_depth, max_depth);
      pass_through.filter (*out);
    }

void
voxel_filter(
        PointCloudConstPtr cloud1,
        PointCloudPtr cloud2,
        float threshold)
{
    VoxeGrid voxel_grid;
    voxel_grid.setInputCloud(cloud1);
    voxel_grid.setLeafSize(threshold, threshold, threshold);
    voxel_grid.filter(*cloud2);
}

void
removeOutliers (
        PointCloudPtr & input,
        PointCloudPtr & out,
        float radius,
        int min_neighbors)
    {
      pcl::RadiusOutlierRemoval<PointType> radius_outlier_removal;
      radius_outlier_removal.setInputCloud (input);
      radius_outlier_removal.setRadiusSearch (radius);
      radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
      radius_outlier_removal.filter (*out);
    }

void
filterCloudStatical(
        PointCloudPtr &cloud_in,
        PointCloudPtr &cloud_out)
{
    pcl::StatisticalOutlierRemoval<PointType> sorSta;
    sorSta.setInputCloud( cloud_in);
    sorSta.setMeanK (50);
    sorSta.setStddevMulThresh (1.0);
    sorSta.filter (*cloud_out);
}



struct regist_trans
    {
        Eigen::Affine3f transform;
        double score;
    };

double trans_error(PointCloudConstPtr target, PointCloudConstPtr source)
    {
        double error = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_z = 0.0;
        for (size_t i = 0; i < source->size(); i++){
            delta_x = source->points[i].x-target->points[i].x;
            delta_y = source->points[i].y-target->points[i].y;
            delta_z = source->points[i].z-target->points[i].z;
            error += (delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z);
        }
        error = sqrt(error/source->size());

        return error;
    }

regist_trans regist_icp( PointCloudConstPtr input, PointCloudConstPtr target)
    {
        regist_trans A;
        A.transform = Eigen::Affine3f::Identity();
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputSource(input);
        icp.setInputTarget(target);
        PointCloudPtr final (new PointCloud);
        icp.align(*final);

        A.transform = icp.getFinalTransformation();
        A.score = icp.getFitnessScore();
        return A;
    }

regist_trans regist_gicp( PointCloudConstPtr input, PointCloudConstPtr target)
    {
        regist_trans A;
        A.transform = Eigen::Affine3f::Identity();
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setInputSource(input);
        gicp.setInputTarget(target);
        PointCloudPtr final (new PointCloud);
        gicp.align(*final);

        A.transform = gicp.getFinalTransformation();
        A.score = gicp.getFitnessScore();
        return A;
    }

double registration( PointCloudPtr scan, PointCloudConstPtr global, PointCloudPtr scan_real, PointCloudConstPtr ground_truth, float grid_value)
    {
        PointCloudPtr tree1Points (new PointCloud);
        PointCloudPtr tree2Points (new PointCloud);
        voxel_filter(scan,tree1Points,grid_value);
        voxel_filter(global,tree2Points,grid_value);

        regist_trans trans_icp1 = regist_icp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_icp1.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_icp1.transform);
        cout << "Score: " << trans_icp1.score << "---";
        cout << "1 Trans_error: " << trans_error(scan_real, ground_truth) << endl;

        voxel_filter(scan,tree1Points,grid_value/8);
        voxel_filter(global,tree2Points,grid_value/8);

        regist_trans trans_icp2 = regist_icp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_icp2.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_icp2.transform);
        cout << "Score: " << trans_icp2.score << "---";
        cout << "2 Trans_error: " << trans_error(scan_real, ground_truth) << endl << endl;

        return trans_icp2.score;
    }

double registration_DICP( PointCloudPtr reg_scan, PointCloudConstPtr pmodel, PointCloudPtr pscan, PointCloudConstPtr ptruth, float grid_value)
    {
        PointCloudPtr tree1Points (new PointCloud);
        PointCloudPtr tree2Points (new PointCloud);

        voxel_filter(reg_scan,tree1Points,grid_value);
        voxel_filter(pmodel,tree2Points,grid_value);

        regist_trans trans_icp = regist_icp(tree1Points, tree2Points);
        pcl::transformPointCloud(*reg_scan,*reg_scan,trans_icp.transform);
        pcl::transformPointCloud(*pscan,*pscan,trans_icp.transform);

//        cout << endl << "Transform matrix is as below: " << endl;
//        cout << trans_icp.transform.affine() << endl;
//        cout << "------------ DICP Trans_error------------: " << trans_error(pscan, ptruth) << " regist Score : " << trans_icp.score << endl << endl;
        return trans_icp.score;
    }

double registration_DGICP( PointCloudPtr scan, PointCloudConstPtr global, PointCloudPtr scan_real, PointCloudConstPtr ground_truth, float grid_value)
    {
        PointCloudPtr tree1Points (new PointCloud);
        PointCloudPtr tree2Points (new PointCloud);

        voxel_filter(scan,tree1Points,grid_value);
        voxel_filter(global,tree2Points,grid_value);

        regist_trans trans_gicp = regist_gicp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_gicp.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_gicp.transform);
        cout << "------------ DGICP Trans_error------------: " << trans_error(scan_real, ground_truth) << " regist Score : " <<
                trans_gicp.score << endl << endl;

        return trans_gicp.score;
    }

double registration_OGICP( PointCloudPtr scan, PointCloudConstPtr global, PointCloudPtr scan_real, PointCloudConstPtr ground_truth, float grid_value)
    {
        PointCloudPtr tree1Points (new PointCloud);
        PointCloudPtr tree2Points (new PointCloud);

        voxel_filter(scan,tree1Points,grid_value);
        voxel_filter(global,tree2Points,grid_value);

        regist_trans trans_icp1 = regist_icp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_icp1.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_icp1.transform);
        cout << "1Trans_error: " << trans_error(scan_real, ground_truth) << endl;

        voxel_filter(scan,tree1Points,grid_value/4);
        voxel_filter(global,tree2Points,grid_value/4);

        regist_trans trans_icp2 = regist_icp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_icp2.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_icp2.transform);
        cout << "2Trans_error: " << trans_error(scan_real, ground_truth) << endl;

        voxel_filter(scan,tree1Points,grid_value/16);
        voxel_filter(global,tree2Points,grid_value/16);

        regist_trans trans_gicp = regist_gicp(tree1Points, tree2Points);
        pcl::transformPointCloud(*scan,*scan,trans_gicp.transform);
        pcl::transformPointCloud(*scan_real,*scan_real,trans_gicp.transform);
        cout << "------------ OGICP Trans_error------------: " << trans_error(scan_real, ground_truth) << "regist Score : " <<
                trans_gicp.score << endl << endl;

        return trans_gicp.score;
    }

regist_trans registration(PointCloudPtr scan, PointCloudPtr model, double grid){

    PointCloudPtr tree1Points (new PointCloud);
    PointCloudPtr tree2Points (new PointCloud);

    voxel_filter(scan,tree1Points,grid);
    voxel_filter(model,tree2Points,grid);

//    cout << "scan has " << tree1Points->size() << " points, model has " << tree2Points->size() << " points"  << endl;

    regist_trans trans_icp = regist_icp(tree1Points, tree2Points);
    return trans_icp;

}

}


#endif
