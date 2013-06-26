// MAIN LOOP
// The main loop allows the user to perform several operation. Basically they are
// 
//  - TRAINING STAGE
//    It deals with object models creation. Every object in the ObjectDB is modeled with a OUR-CVFH histogram for each view of the object itself. 
//    The training stage is structured as follows:
//      T1. Point cloud acquisition from Kinect
//      T2. Pass.through filter application
//      T3. Dominant plane and clusters extraction. 
//          T4. Cluster visualization and storage.
//              For each found cluster it is visualized and the user is asked to choose to save it or not. If the choice is yes:
//              T4.1. MLS filter application and normals estimation
//              T4.2. OUR-CVFH signature estimation
//              T4.3. Cloud storage
//      T5. Kd-Trees update. 
//          For each folder updated in previous steps
//          T5.1. OUR-CVFH histogram conversion to Flann format
//          T5.2. OUR-CVFH histograms path add
//          T5.3. Kd-Tree index build and storage
// 
//  - NEW SCENE CREATION STAGE
//    It deals with the creation of new reference scene. Every new scene is saved in the SceneDB. 
//    The newscene stage is structured as follows:
//      NS1. Point cloud acquisition form Kinect
//      NS2. Pass-through filter application
//      NS3. Dominant plane and clusters extraction
//           NS.4. Clusters OUR-CVFH signature determination and DB search
//                 For each found cluster:
//                 NS.4.1. MLS filter application and normals estimation
//                 NS.4.2. OUR-CVFH signature estimation
//                         NS.4.2.1. OUR-CVFH signature transformation
//                 NS.4.3. DB search
//                         If a new model is detected and the user chooses to save it
//                         NS.4.3.1. OUR-CVFH histogram conversion to Flann format
//                         NS.4.3.2. OUR-CVFH histogram path add
//                         NS.4.3.3. Kd-Tree index build and storage
//      NS5. Scene storage
//      NS6. Scene visualization
// 
//  - KUKAVISION STAGE
//    It deals with the acquisition of a modified scene and the comparation between it and a selected reference scene. At last this step produces all transformation
//    that the Kuka arm has to perform. 
//    The kukavision step is structure as follows:
//      KV1. Reference scene selection
//      KV2. Point cloud acquisition from Kinect
//      KV3. Pass-through filter application
//      KV4. Dominant plane and clusters extraction
//           For each found cluster:
//           KV5. Clusters OUR-CVFH signature estimation and DB search
//                KV5.1. MLS filter application and normals estimation
//                KV5.2. OUR-CVFH signature estimation
//                       KV5.2.1. OUR-CVFH signature transformation
//                KV5.3. DB search

#include <pcl/console/parse.h>
#include "MainLoop.h"

int 
main(int argc, char** argv)
{
  MainLoop main_loop;
  bool training_(false), new_ref_scene_(false), kuka_vision_(false);

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
  else if (pcl::console::find_argument (argc, argv, "-kukavision") >= 0)
    kuka_vision_ = true;
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
  else if(kuka_vision_)
    main_loop.KukaVisionStage();

  return(0);
}