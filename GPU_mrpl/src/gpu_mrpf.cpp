#include "gpu_mrpf.h"
#include "filter.h"

using namespace gpu_mrpf;
using namespace pcl;
using namespace std;

GpuMRPF::GpuMRPF(int &ac, char **&av)
    : ac_ (ac),
      av_ (av)
{
    // Initialization.
    initial();
    readParameters();
    printParameters();
    preFilter();


}

void GpuMRPF::setupParticles()
{
    voxelFitler(Scan_, TempScan_, init_grid_);
    voxelFitler(Map_, TempMap_, init_grid_);

    Ps_->clear();

    for (int i = 0; i < num_particles_; i++)
    {

        int index = rand()%(TempMap_->points.size()-1);
        Particle pt;
        pt.point = TempMap_->points[index];
        pt.heading = 0.0;
        pt.weight  = 0;
        pt.trans   = Eigen::Affine3f::Identity();
        Ps_->push_back(pt.point);

        particles_.push_back(pt);
    }

    thrust::host_vector<double> dparticles(10);
}

void GpuMRPF::update()
{

}


void GpuMRPF::update_model()
{

}


void GpuMRPF::resample()
{

}
