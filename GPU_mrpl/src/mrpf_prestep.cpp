#include "common.h"
#include "gpu_mrpf.h"
#include "filter.h"

using namespace gpu_mrpf;
using namespace pcl;
using namespace std;

bool GpuMRPF::initial()
{
    time_cost_ = 0.0;

    Scan_       = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    Map_        = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    TempScan_   = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    TempMap_    = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    Reg_        = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    Ips_        = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    Ps_         = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);

    strScan_            = "Scan.pcd";
    strMap_             = "Map.pcd";

    init_grid_          = 0.5;
    eRate_              = 0.1;
    divider_            = 0.2;
    voxel_filter_       = 0.2;
    num_particles_      = 1000;

    translation_.x      = 0.0;
    translation_.y      = 0.0;
    translation_.z      = 0.0;
    heading_            = 0.0;

    initial_trans_      = Eigen::Affine3f::Identity();

    return 0;
}

bool GpuMRPF::readParameters()
{
    cout << endl << endl;
    console::print_info("loading data...\n\n");

    strScan_ = av_[1];
    strMap_  = av_[2];
    pcl::io::loadPCDFile(strScan_, *Scan_);
    pcl::io::loadPCDFile(strMap_, *Map_);

    pcl::console::parse_argument(ac_, av_, "-g", init_grid_);
    pcl::console::parse_argument(ac_, av_, "-e", eRate_);
    pcl::console::parse_argument(ac_, av_, "-d", divider_);
    pcl::console::parse_argument(ac_, av_, "-voxel", voxel_filter_);
    pcl::console::parse_argument(ac_, av_, "-p", num_particles_);
    particles_.resize(num_particles_);

    pcl::console::parse_3x_arguments(ac_, av_, "-t", translation_.x, translation_.y, translation_.z);
    pcl::console::parse_argument(ac_, av_, "-heading", heading_);

    initial_trans_.translation() << translation_.x, translation_.y, translation_.z;
    initial_trans_.rotate(Eigen::AngleAxisf (heading_, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*Scan_, *Scan_, initial_trans_);

    return 0;
}

bool GpuMRPF::printParameters()
{
    pcl::console::print_color(stdout, 1, 3, "The input parameters are as folows.\n");

    pcl::console::print_highlight("Input Scan is %s with point size %d.\n", strScan_.c_str(), Scan_->size());
    pcl::console::print_highlight("Input Map is %s with point size %d.\n", strMap_.c_str(), Map_->size());

    pcl::console::print_highlight("Initial grid size is %0.2f.\n", init_grid_);
    pcl::console::print_highlight("Error threshold is %0.2f.\n", eRate_);
    pcl::console::print_highlight("Divider is %0.2f.\n", divider_);
    pcl::console::print_highlight("Particle filter number is %d.\n", num_particles_);

    pcl::console::print_highlight("Initial translation error is %0.2f, %0.2f, %0.2f.\n", translation_.x, translation_.y, translation_.z);
    pcl::console::print_highlight("Initial heading error is %0.2f.\n", heading_);

    return 0;
}

void GpuMRPF::preFilter()
{
    voxelFitler(Scan_, Scan_, voxel_filter_);
    statisticFilter(Scan_, Scan_, 50, 1.0);
    radiusFilter(Scan_, Scan_, 1, 5);

    *Reg_ = *Scan_;
}

