#ifndef GPU_MRPF_H
#define GPU_MRPF_H

#include "common.h"

namespace gpu_mrpf {

struct Particle{

    PointT            point;
    Eigen::Affine3f   trans;
    double            heading;
    double            weight;

//    Particle()
//    {
//        point.x = 0.0;
//        point.y = 0.0;
//        point.z = 0.0;

//        trans = Eigen::Affine3f::Identity();
//        heading = 0.0;
//        weight  = 0.0;
//    }

//    Particle(PointT point_) :point (point_){}

    __host__ __device__
        bool operator<(const Particle other) const
    {
        return weight < other.weight;
    }
};

class GpuMRPF{

public:

    /*!
     * Constructor.
     */
    GpuMRPF(int &ac, char **&av);

    /*!
     * Deconstructor.
     */
    ~GpuMRPF() {}

    /*!
     * Setup particles.
     */
    void setupParticles();

    /*!
     * Update the particles.
     */
    void update();

    /*!
     * Update the particle model.
     */
    void update_model();

    /*!
     * Resample from the post distribution.
     */
    void resample();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr TempScan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr TempMap_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Reg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Ips_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Ps_;

private:

    /*!
     * Initial the mrpf model.
     */
    bool initial();

    /*!
     * Read the Parameters.
     */
    bool readParameters();

    /*!
     * Print inputed parameters.
     */
    bool printParameters();

    /*!
     * Prepocess filtering step.
     */
    void preFilter();


    //! main parameters.
    int ac_;
    char** av_;

    //! map parameters.
    double init_grid_;
    double eRate_;
    double divider_;
    double voxel_filter_;
    int num_particles_;

    //! initial error.
    PointT translation_;
    double heading_;
    Eigen::Affine3f initial_trans_;

    //! PCL time.
    pcl::console::TicToc time_;
    double time_cost_;

    //! map & scan
    std::string strScan_;
    std::string strMap_;

    //! estimate particles
    Particle    estimate_trans_;

    //! thrust for particles.
    thrust::host_vector<Particle> particles_;
};
}

#endif // GPU_MRPF_H
