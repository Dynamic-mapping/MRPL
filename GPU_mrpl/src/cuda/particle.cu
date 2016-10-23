#include <iostream>

// PCL COMMON
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/point_cloud.h>
// CUDA STL
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>

//using namespace std;

int particle(void)
{
    pcl::cuda::PointCloudAOS<pcl::cuda::Host> a;

    size_t N = 10;
    thrust::device_vector<double> X(N, 1);

    std::cout << X.size() << std::endl;
    return 0;
}
