// MAIN LOOP
// The main loop allows the user to perform several operation. Basically they are
// 
//  - TRAINING STAGE
//    It deals with object models creation. Every object in the DB is modeled with a OUR-CVFH histogram for each view of the object itself. 
//    The training stage is structured as follows:
//      T1. Acquisition of the cloud from Kinect
//      T2. Dominant plane and clusters extraction. For each found cluster:
//          T2.1. Visualize the cluster and choose if it has to be saved. If yes:
//                T2.1.1. Apply a voxel grid to uniform the point cloud density
//                T2.1.2. Compute normals of the cluster
//                T2.1.3. Compute OUR-CVFH signature
//                T2.1.4. Save point cloud and OUR-CVFH signature
//      T3. Kd-Trees update. For each folder updated in previous steps
//          T3.1. Convert OUR-CVFH histogram to Flann format and save it
//          T3.2. Save the list of path corresponding to all elements of the current subfolder
//          T3.3. Create, build and save a Kd-Tree 
// 
//  - NEW SCENE CREATION STAGE
//  - RECOGNITION STAGE

#include <pcl/console/parse.h>
#include "MainLoop.h"

int 
main(int argc, char** argv)
{
  MainLoop main_loop;
  bool training_(false), new_ref_scene_(false), cluster_recognition(false);

  // PARSING THE COMMAND LINE
	if (pcl::console::find_argument (argc, argv, "-h") >= 0)
 	{
 		main_loop.PrintUsage (argv[0]);
    return 0;
  }
  else if (pcl::console::find_argument (argc, argv, "-training") >= 0)
    training_ = true;
  else if (pcl::console::find_argument (argc, argv, "-newscene") >= 0)
    new_ref_scene_ = true;
  else
  {
   	main_loop.PrintUsage (argv[0]);
   	return 0;
 	}

  // Parsing the line
  if(training_)
    main_loop.TrainingStage();
  else if(new_ref_scene_)
    main_loop.NewSceneStage();

  return(0);
}