#include "gpu_mrpf.h"

using namespace std;
using namespace pcl;
using namespace gpu_mrpf;

bool next_loop;

void keyboardEventOccurred(const visualization::KeyboardEvent &event, void *nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        next_loop = true;
}

void print_meau()
{
    pcl::console::print_info ("Syntax is: ./gpu_mrpf_node Scan.pcd Model.pcd <options>\n");
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -g initial grid ...................................Grid size\n");
    pcl::console::print_info ("    -e error  .................................. Allowable error\n");
    pcl::console::print_info ("    -d divider  .......................... divider for particles\n");
    pcl::console::print_info ("    -voxel voxel_filter  ......................voxel filter size\n");
    pcl::console::print_info ("    -p num_particles ........................... particle number\n");
    pcl::console::print_info ("    -t trans_x, trans_y, trans_z ..............Translation error\n");
    pcl::console::print_info ("    -heading heading ............................ Rotation error\n");
    pcl::console::print_info ("    -re resolution ........................ Resolution of global\n");
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        print_meau();
        return 1;
    }

    GpuMRPF gpuMRPF (argc, argv);

    /*============================================================================================*/
    /*                                     Visual setup                                           */
    /*============================================================================================*/

    pcl::visualization::PCLVisualizer viewer("MRPF");

    viewer.registerKeyboardCallback( &keyboardEventOccurred, (void*) NULL);
    viewer.setBackgroundColor( 255, 255, 255 );

    CustomColor vis_Map             ( gpuMRPF.Map_,      255, 0, 0    );
    CustomColor vis_Reg             ( gpuMRPF.Reg_,      0, 0, 255    );
    CustomColor vis_Particle        ( gpuMRPF.Ps_,       0, 255, 0    );
    CustomColor vis_IMParticle      ( gpuMRPF.Ips_,      255, 255, 0  );

    viewer.addPointCloud( gpuMRPF.Map_,      vis_Map,            "vis_Map"           );
    viewer.addPointCloud( gpuMRPF.Reg_,      vis_Reg,            "vis_Reg"           );
    viewer.addPointCloud( gpuMRPF.Ps_,       vis_Particle,       "vis_Particle"      );
    viewer.addPointCloud( gpuMRPF.Ips_,      vis_IMParticle,     "vis_IMParticle"    );

    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             "vis_Map"       );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,
                                             "vis_Reg"         );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,
                                             "vis_Particle"    );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,
                                             "vis_IMParticle"  );
    /** ============================================================================================
    /* ================================= Registration process ======================================
    /*      @ Use Particle filter to sample from different points on a corsa map.
    /*============================================================================================*/

    timer t;

    gpuMRPF.setupParticles();

    while (!viewer.wasStopped()){

        viewer.spinOnce();

        if (next_loop){

            /** ===============================================================================
             *                         Step 1:   Particles update
             *  Update particle weight according to how good every particle matches, Use the
             *  icp_fitness_score to measure the fitness
            /*  =============================================================================*/

            gpuMRPF.update();

            viewer.updatePointCloud( gpuMRPF.Map_,    vis_Map,             "vis_Map"        );
            viewer.updatePointCloud( gpuMRPF.Reg_,    vis_Reg,             "vis_Reg"        );
            viewer.updatePointCloud( gpuMRPF.Ps_,     vis_Particle,        "vis_Particle"   );
            viewer.updatePointCloud( gpuMRPF.Ips_,    vis_IMParticle,      "vis_IMParticle" );

            ///==============================================================================///


            /** ===============================================================================
             *                         Step 2:   Coarse to Fine
             *  Deeper resolution or break out
             *  THis is the key point of my Method
            /*  =============================================================================*/

            gpuMRPF.update_model();

            ///==============================================================================///


            /** ===============================================================================
             *                         Step 3:   Importance Resample
             *  Resample to new particles
             *  Show the new paticles set on map
            /*  =============================================================================*/

            gpuMRPF.resample();

            ///==============================================================================///
        }
        next_loop = false;
    }


    return 0;
}
