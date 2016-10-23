#include <pf_mrpf.h>
#include <iostream>
#include <algorithm>
#include <pcl/console/time.h>
#include <math.h>

using namespace std;
using namespace Eigen;
using namespace pf_mrpf;

PointCloudPtr pScan         (new PointCloud);
PointCloudPtr pModel        (new PointCloud);
PointCloudPtr pReg          (new PointCloud);
PointCloudPtr pLocal_Model  (new PointCloud);

PointCloudPtr pParticle     (new PointCloud);
PointCloudPtr   pIMParticle   (new PointCloud);



bool next_loop = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                           void* nothing)
{
    if( event.getKeySym() == "space" && event.keyDown())
                next_loop = true;
}

int main( int argc, char** argv){

    /* ============================================================================================*/
    /*                                  1.Input information                                        */
    /* ============================================================================================*/
    if (argc < 3)
    {
      pcl::console::print_info ("Syntax is: %s Scan.pcd Model.pcd <options>\n", argv[0]);
      pcl::console::print_info ("  where options are:\n");
      pcl::console::print_info ("    -g initial grid ...................................Grid size\n");
      pcl::console::print_info ("    -e error  .................................. Allowable error\n");
      pcl::console::print_info ("    -s file_name ....................... Save output information\n");
      pcl::console::print_info ("    -d divider  .......................... divider for particles\n");
      pcl::console::print_info ("    -p num_particles ........................... particle number\n");
      pcl::console::print_info ("    -t trans_x, trans_y, trans_z ..............Translation error\n");
      pcl::console::print_info ("    -r rot_x, rot_y, rot_z ...................... Rotation error\n");
      pcl::console::print_info ("    -re resolution ........................ Resolution of global\n");
      pcl::console::print_info ("    -pcd pcd_name .............................. Save output pcd\n");
      return (1);
    }
    /*=============================================================================================*/
    /*============================================================================================*/


    /* ============================================================================================*/
    /*                                  2.Parameter setup                                          */
    /* ============================================================================================*/
    int     time_ms         = 0   ;
    double   init_grid       = 10  ;
    double  num_particles   = 100 ;
    double  divider         = 3.0 ;
    double  eRate           = 0.1 ;
    string  file_name;
    string  pcd_name;

    float trans_x = 0.0, trans_y=0.0, trans_z=0.0;
    float rot_z=0.0;

    pcl::console::print_info ("   loading data ...\n");
    pcl::console::TicToc time;
    pcl::io::loadPCDFile(argv[1],   *pScan);
    pcl::io::loadPCDFile(argv[2],   *pModel);
    pcl::console::parse_argument (argc, argv, "-g",     init_grid       );
    pcl::console::parse_argument (argc, argv, "-e",     eRate           );
    pcl::console::parse_argument (argc, argv, "-d",     divider         );
    pcl::console::parse_argument (argc, argv, "-p",     num_particles   );
    pcl::console::parse_argument (argc, argv, "-s",     file_name       );
    pcl::console::parse_argument (argc, argv, "-pcd",   pcd_name        );

    pcl::console::parse_3x_arguments(argc, argv, "-t", trans_x, trans_y, trans_z);
    pcl::console::parse_argument(argc, argv, "-heading", rot_z);

    /*============================================================================================*/
    /*============================================================================================*/


    /*============================================================================================*/
    /*                                  3. Preprocess                                             */
    /*============================================================================================*/

    // Filter
    voxel_filter(pScan, pScan, 0.2);
    filterCloudStatical(pScan, pScan);
    removeOutliers(pScan, pScan, 1, 5);

    Eigen::Affine3f initial_transform = Eigen::Affine3f::Identity();
    initial_transform.translation() << trans_x, trans_y, trans_z;
    initial_transform.rotate(Eigen::AngleAxisf (rot_z, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*pModel, *pModel, initial_transform);
    /*============================================================================================*/
    /*============================================================================================*/


    /*============================================================================================*/
    /*                                  4. Visual setup                                           */
    /*============================================================================================*/

    pcl::visualization::PCLVisualizer viewer("MRPF");

    viewer.registerKeyboardCallback( &keyboardEventOccurred, (void*) NULL);
    viewer.setBackgroundColor( 255, 255, 255 );

    CustomColor vis_Model           ( pLocal_Model,    255, 0, 0    );
    CustomColor vis_Reg             ( pReg,            0, 0, 255    );
    CustomColor vis_Particle        ( pParticle,       0, 255, 0    );
    CustomColor vis_IMParticle      ( pIMParticle,     255, 255, 0  );

    *pLocal_Model = *pModel;
    *pReg         = *pScan;


    viewer.addPointCloud( pLocal_Model,   vis_Model,          "vis_Model"         );
    viewer.addPointCloud( pReg,           vis_Reg,            "vis_Reg"           );
    viewer.addPointCloud( pParticle,      vis_Particle,       "vis_Particle"      );
    viewer.addPointCloud( pParticle,      vis_IMParticle,     "vis_IMParticle"    );


    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,     "vis_Model"       );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,     "vis_Reg"         );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,     "vis_Particle"    );
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,     "vis_IMParticle"  );
    /*============================================================================================*/
    /*============================================================================================*/


    /** ============================================================================================
    /* ================================= Registration process ======================================
    /*      @ Use Particle filter to sample from different points on a corsa map.
    /*============================================================================================*/


    /** ======================================================================================
    /*                                 Step 0:   Initial Particles
    /*  ====================================================================================*/
    Particles PTs;
    PTs.setup(pScan, pModel, num_particles, divider, init_grid, initial_transform);

    while (!viewer.wasStopped()){

        viewer.spinOnce();

        if (next_loop){

            /** ===============================================================================
             *                         Step 1:   Particles update
             *  Update particle weight according to how good every particle matches, Use the
             *  icp_fitness_score to measure the fitness
            /*  =============================================================================*/

            time.tic();
            PTs.update();
            time_ms += time.toc();

            *pLocal_Model = *PTs.pTemp_model_;
            *pReg         = *PTs.pReg_;
            *pParticle    = *PTs.pParticle_;
            *pIMParticle  = *PTs.pIMParticle_;

            viewer.updatePointCloud( pLocal_Model,    vis_Model,          "vis_Model"      );
            viewer.updatePointCloud( pReg,           vis_Reg,             "vis_Reg"        );
            viewer.updatePointCloud( pParticle,      vis_Particle,        "vis_Particle"   );
            viewer.updatePointCloud( pIMParticle,  vis_IMParticle,        "vis_IMParticle" );
            ///==============================================================================///


            /** ===============================================================================
             *                         Step 2:   Coarse to Fine
             *  Deeper resolution or break out
             *  THis is the key point of my Method
            /*  =============================================================================*/

            time.tic();
            PTs.update_model();
            time_ms += time.toc();

            PTs.output();
            cout << PTs.grid_level() << endl;
            if (PTs.grid_level() <= 0.7 ) {
                 cout << "time total used is " << time_ms << endl;
                 viewer.spin();
                 break;
            }

            ///==============================================================================///


            /** ===============================================================================
             *                         Step 3:   Importance Resample
             *  Resample to new particles
             *  Show the new paticles set on map
            /*  =============================================================================*/

            time.tic();
            PTs.resample();
            time_ms += time.toc();

            pcl::console::print_color(stdout, 1, 3, "current time cost: %d ms\n", time_ms);
            ///==============================================================================///
        }
        next_loop = false;
    }
    return 0;
}
