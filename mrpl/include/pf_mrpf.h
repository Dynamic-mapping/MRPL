#ifndef PF_MRPF_H
#define PF_MRPF_H

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <algorithm>
#include <math.h>


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

using namespace std;
using namespace Eigen;

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

regist_trans registration(PointCloudPtr scan, PointCloudPtr model, double grid){

    PointCloudPtr tree1Points (new PointCloud);
    PointCloudPtr tree2Points (new PointCloud);

    voxel_filter(scan,tree1Points,grid);
    voxel_filter(model,tree2Points,grid);

    regist_trans trans_icp = regist_icp(tree1Points, tree2Points);
    return trans_icp;

}

namespace pf_mrpf{

struct Eval_value{
    double E_trans;
    double E_rot;
};

Eval_value Evaluation(Matrix4f est_mat, Matrix4f err_mat){
    Eval_value E;
    E.E_trans = sqrt(pow(est_mat(0,3)-err_mat(0,3),2)+pow(est_mat(1,3)-err_mat(1,3),2)+pow(est_mat(2,3)-err_mat(2,3),2));
    E.E_rot = (acos((est_mat(0,0)+est_mat(1,1)+est_mat(2,2)-1)*0.5) - acos((err_mat(0,0)+err_mat(1,1)+err_mat(2,2)-1)*0.5))*180/M_PI;
    return E;
}

struct Model{
    size_t index;
    double distance;
};

bool comp_model(const Model &a, const Model &b){
    return a.distance < b.distance;
}

struct Particle{

    PointType         point;
    Eigen::Affine3f   trans;
    double            heading;
    double            weight;
};

bool comp_particle(const Particle &a, const Particle &b){
    return a.weight > b.weight;
}

class Particles{

public:

    PointCloudPtr       pTemp_model_  ;
    PointCloudPtr       pTemp_scan_   ;
    PointCloudPtr       pParticle_    ;
    PointCloudPtr       pIMParticle_  ;
    PointCloudPtr       pReg_         ;
    PointCloudPtr       pModel_;
    PointCloudPtr       pScan_;

     Particles():
         pTemp_model_ (new PointCloud),
         pTemp_scan_ (new PointCloud),
         pParticle_ (new PointCloud),
         pIMParticle_ (new PointCloud),
         pReg_ (new PointCloud),
         pModel_ (new PointCloud),
         pScan_ (new PointCloud)
     {
        num_particles_    = 100;
        divider_          = 3;
        sample_grid_      = 10.0;
        var_particle_     = 0.0;
        initial_trans_    = Eigen::Affine3f::Identity();
    }

    ~Particles(){}

    double grid_level()
    {
        return sample_grid_;
    }

    void setup (PointCloudPtr pScan,
                PointCloudPtr pModel,
                int num_particles,
                int divider,
                int grid,
                Eigen::Affine3f initial_trans) {

        num_particles_ = num_particles;
        divider_ = divider;
        sample_grid_ = grid;
        initial_trans_ = initial_trans;
        *pScan_     =   *pScan;
        *pModel_    =   *pModel;

        voxel_filter(pScan_, pTemp_scan_, sample_grid_);
        voxel_filter(pModel_, pTemp_model_, sample_grid_);

        map_random_generate();
    }

    /** @brief set the model pointcloud according to:
     *  1. current Model
     *  2. current estimate position
     *  3. current grid level
     *  Deeper the grid level, estimate the submap at
     *  the center of estimate_ pose, output this map
     *  as new map
     */
    void update_model ()
    {

        var_particle_ = 0.0;
        for (size_t i = 0; i < particles_.size(); i ++)
            var_particle_ += pow((particles_[i].point.x - estimate_.point.x),2)
                          + pow((particles_[i].point.y - estimate_.point.y),2)
                          + pow((particles_[i].point.z - estimate_.point.z),2);

        var_particle_ = sqrt(var_particle_/particles_.size());

        if (var_particle_ <= 8*sample_grid_){

            sample_grid_ /= 2;
            num_particles_ /= 2;
            voxel_filter(pModel_, pTemp_model_, sample_grid_);
            voxel_filter(pScan_, pTemp_scan_, sample_grid_);

        }
    }

    void add(const Particle pt, int Noise = false)
    {

        Particle new_pt;

        new_pt.point.x = pt.point.x;
        new_pt.point.y = pt.point.y;
        new_pt.point.z = pt.point.z;
        new_pt.heading = pt.heading;
        new_pt.weight = -1;

        if (Noise == false){
            new_pt.point.x += ((rand()%100)-50)*sample_grid_/100.0;
            new_pt.point.y += ((rand()%100)-50)*sample_grid_/100.0;
            new_pt.point.z += ((rand()%100)-50)*sample_grid_/100.0;
            new_pt.heading += ((rand()%100)-50)*M_PI/5000.0;
        }
        else{
            new_pt.point.x += ((rand()%100)-50)*sample_grid_/50.0;
            new_pt.point.y += ((rand()%100)-50)*sample_grid_/50.0;
            new_pt.point.z += ((rand()%100)-50)*sample_grid_/200.0;
            new_pt.heading += ((rand()%100)-50)*M_PI/500.0;
        }
        new_pt.trans = Eigen::Affine3f::Identity();
        new_pt.trans.translation() << pt.point.x, pt.point.y, pt.point.z;
        new_pt.trans.rotate(Eigen::AngleAxisf (pt.heading, Eigen::Vector3f::UnitZ()));

        particles_.push_back(new_pt);
    }

    void map_random_generate()
    {

        for (int i = 0; i < num_particles_; i++){

            Particle pt;

            int index = rand()%(pModel_->points.size()-1);
            pt.point = pModel_->points[index];
            pt.weight = -1;
            pt.trans = Eigen::Affine3f::Identity();

            for (int i = 0 ; i < 6; i++){

                double direction = M_PI*(1.0-i/3.0);
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << pt.point.x, pt.point.y, pt.point.z;
                transform.rotate(Eigen::AngleAxisf (direction, Eigen::Vector3f::UnitZ()));
                PointCloudPtr pTemp (new PointCloud);

                pcl::transformPointCloud(*pTemp_scan_, *pTemp, transform);


                regist_trans new_regist = registration(pTemp, pTemp_model_, sample_grid_);
                double weight = 1/new_regist.score;

                if (pt.weight < weight){

                    pt.weight = weight;
                    pt.heading = direction;
                    pt.trans = new_regist.transform*transform;

                    Matrix4f new_trans = pt.trans.matrix();
                    pt.point.x = new_trans(0,3);
                    pt.point.y = new_trans(1,3);
                    pt.point.z = new_trans(2,3);
                }

            }

            add(pt, true);
        }
    }

    void update()
    {

        double sum_nom = 0.0;
        for (size_t i = 0; i < particles_.size(); i++){

            cout << particles_[i].point.x << " " << particles_[i].point.y << " "<<particles_[i].point.z << endl;
            PointCloudPtr pTemp (new PointCloud);

            pcl::transformPointCloud(*pTemp_scan_, *pTemp, particles_[i].trans);
            regist_trans new_regist = registration(pTemp, pTemp_model_, sample_grid_);

            // update particles according to the registration
//            particles_[i].trans = new_regist.transform * particles_[i].trans;
//            Matrix4f new_trans = particles_[i].trans.matrix();

//            particles_[i].point.x       = new_trans(0,3);
//            particles_[i].point.y       = new_trans(1,3);
//            particles_[i].point.z       = new_trans(2,3);
//            particles_[i].heading = asin(new_trans(1,0));
//            cout << "heading " << particles_[i].heading << endl;

            particles_[i].weight = 1/new_regist.score; //! TODO
            sum_nom += particles_[i].weight;

        }
        cout << endl << endl;
        for (size_t i = 0; i < particles_.size(); i++){
            cout << particles_[i].point.x << " " << particles_[i].point.y << " "<<particles_[i].point.z << endl;
        }


        sort(particles_.begin(), particles_.end(), comp_particle);


        // estimate pose update
        estimate_.point.x = 0;
        estimate_.point.y = 0;
        estimate_.point.z = 0;
        estimate_.heading = 0;
        estimate_.weight = 0;
        estimate_.trans = Eigen::Affine3f::Identity();

        pParticle_->clear();
        pIMParticle_->clear();

        for (size_t i = 0; i < particles_.size(); i++){

            if (i < particles_.size()/divider_){
                pIMParticle_->push_back(particles_[i].point);
            }
            else{
                pParticle_->push_back(particles_[i].point);

            }

            particles_[i].weight /= sum_nom;

            if ( i < particles_.size()/divider_ ){
                estimate_.point.x += particles_[i].point.x;
                estimate_.point.y += particles_[i].point.y;
                estimate_.point.z += particles_[i].point.z;
                estimate_.heading += particles_[i].heading;
                estimate_.weight  += particles_[i].weight;
            }

        }

        estimate_.point.x /= (particles_.size()/divider_);
        estimate_.point.y /= (particles_.size()/divider_);
        estimate_.point.z /= (particles_.size()/divider_);
        estimate_.heading /= (particles_.size()/divider_);
        estimate_.trans.translation() << estimate_.point.x, estimate_.point.y, estimate_.point.z;
        estimate_.trans.rotate(Eigen::AngleAxisf (estimate_.heading, Eigen::Vector3f::UnitZ()));


        pcl::transformPointCloud(*pScan_, *pReg_, estimate_.trans);

        PointCloudPtr new_map (new PointCloud);
        for (size_t i = 0; i < pTemp_model_->points.size(); i++){

            double distance = sqrt(pow(estimate_.point.x-pTemp_model_->points[i].x, 2)
                                 + pow(estimate_.point.y-pTemp_model_->points[i].y, 2));
            double minial = 20*sample_grid_ > 30 ? 20*sample_grid_ : 30;

            if( distance <= minial)
                new_map->push_back(pTemp_model_->points[i]);
        }
        pTemp_model_ = new_map;
    }

    void resample()
    {
        vector<Particle>    new_particles = particles_;
        particles_.clear();
        for (size_t i = 0; i < new_particles.size()/divider_; i++){
            for (size_t j = 0; j < num_particles_ * new_particles[i].weight/estimate_.weight; j++){
                add(new_particles[i]);
            }
        }

    }
    void output(){

        regist_trans estimate_trans = registration(pReg_, pModel_, sample_grid_);
        Eval_value eval_PF = Evaluation(estimate_.trans.matrix(), initial_trans_.matrix());

        cout << endl << endl << "================= current status ================== " << endl;
        cout << "estimate-> x: "    << estimate_.point.x    << " y: "       << estimate_.point.y    << " z: "   << estimate_.point.z    << endl;
        cout << "heading: "         << estimate_.heading    << " weight: "  << estimate_.weight     << endl;
        cout << "Current grid : "   << sample_grid_         << endl;
//        cout << "particle size is " << particles_.size()    << endl;
//        cout << "current var particle is "                  << var_particle_ << endl;
        cout << "new pTemp model has points "               << pTemp_model_->points.size()               << endl;
        cout << "ICP error: "       << estimate_trans.score << endl;
        cout << initial_trans_.matrix() << endl << endl;
        cout << estimate_.trans.matrix() << endl << endl;

        cout << "curretn icp trans " << endl;
        cout << estimate_trans.transform.matrix() << endl;

        cout << "adjust trans " << endl;
        cout << estimate_trans.transform.matrix() * estimate_.trans.matrix() <<endl;



        pcl::console::print_color(stdout, 1, 3, "Trans error : %0.2f m, Rotation error : %f deg\n", eval_PF.E_trans, eval_PF.E_rot);
        pcl::console::print_color(stdout, 1, 3, "variance of important particle is %f.\n", var_particle_);
    }


private:
    int         num_particles_;
    int         divider_;
    double      sample_grid_;
    double      var_particle_;

    vector<Particle>    particles_;
    Particle            estimate_;
    Eigen::Affine3f     initial_trans_;



};

}

#endif // PF_MRPF_H
