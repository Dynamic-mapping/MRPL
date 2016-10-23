cmake_minimum_required (VERSION 2.8)
project(GPU_MRPF)
#set(CMAKE_CXX_FLAGS "-std=c+11 ${CMAKE_CXX_FLAGS}")
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++11)
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_10,code=sm_10)

find_package(CUDA REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_TOOLKIT_INCLUDE}
    ${Boost_LIBRARY_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

cuda_add_library(gpu_particle     src/particle.cu)
target_link_libraries(gpu_particle
    pcl_cuda_io
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(../gpu_mrpf_node
    src/gpu_mrpf_node.cpp
    src/gpu_mrpf.cpp
    src/mrpf_prestep.cpp
    include/gpu_mrpf.h
    include/filter.h
    include/filter.cpp
    include/common.h
    include/timer.h

)
target_link_libraries(../gpu_mrpf_node
    pcl_cuda_io
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
)
