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
//  - TRAINING STAGE MESH
//    In this stage the objects are not acquired by Kinect sensor but they are taken from a virtual scannning around synthetic meshes.
//    The output of the function is the same but there is no need to consider a grabber and step T4.3. is no more performed (since the 
//    virtual scanning automatically saves .pcd files when terminate)
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
//      KV6. Infos display
//           KV6.1. Inner and outer objects infos screen print
//           KV6.2. Inner and outer objects visualization in the actual scene
// 
//    MAINTENANCE STAGE
//    In this stage the user can delete a reference scene or an object model stored in the one of the two DBs

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include "TrainingStage.h"
#include "TrainingStageMesh.h"
#include "NewSceneStage.h"
#include "KukaVisionStage.h"
#include "MaintenanceStage.h"

void
PrintUsage(const char* progName)
{
  std::cout << std::endl << std::endl << "Usage: "<<progName<<" [options]" << std::endl << std::endl
            << "Options:" << std::endl
              << "-------------------------------------------" << std::endl
              << "-h            THIS HELP" << std::endl
              << "-t            TRAINING STAGE" << std::endl
              << "              Object model creation (Kinect acquisition of the cluster, OUR-CVFH descriptor determination" << std::endl
              << "              and KdTree update)" << std::endl
              << "-tm           TRAINING STAGE MESH" << std::endl
              << "              Object model creation (Virtual scanning of synthetic meshes, OUR-CVFH descriptor determination) " << std::endl
              << "              and KdTree update)" << std::endl
              << "-n            NEW SCENE REGISTRATION STAGE" << std::endl
              << "              New reference scene acquisition" << std::endl
              << "-k            KUKAVISION STAGE" << std::endl
              << "              Actual scene acquisition and objects pose transformations determination" << std::endl
              << "-m            MAINTENANCE STAGE" << std::endl
              << "              Delete a reference scene or an object model from DB"
              << std::endl;
};

int 
main(int argc, char** argv)
{
  bool training_(false), training_mesh_(false), new_ref_scene_(false), kuka_vision_(false), maintenance_(false);

  // PARSING THE COMMAND LINE
	if (pcl::console::find_argument (argc, argv, "-h") >= 0)
 	{
 		PrintUsage (argv[0]);
    return 0;
  }
  else if (pcl::console::find_argument (argc, argv, "-t") >= 0)
    training_ = true;
  else if (pcl::console::find_argument (argc, argv, "-tm") >= 0)
    training_mesh_ = true;
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
    new_ref_scene_ = true;
  else if (pcl::console::find_argument (argc, argv, "-k") >= 0)
    kuka_vision_ = true;
  else if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    maintenance_ = true;
  else
  {
   	PrintUsage (argv[0]);
   	return 0;
 	}

  // Parsing the line
  if(training_)
  {
    // Class instantiation
    TrainingStage training;
    // Stage running    
    training.RunStage();
  }
  else if (training_mesh_)
  {
    // Class instantiation
    TrainingStageMesh training_mesh;
    // Stage running
    training_mesh.RunStage();
  }
  else if (new_ref_scene_)
  {
    // Class instantiation
    NewSceneStage newscene;
    // Stage running
    newscene.RunStage();
  }
  else if (kuka_vision_)
  {
    // Class instantiation
    KukaVisionStage kukavision;
    // Stage running
    kukavision.RunStage();
  }
  else if (maintenance_)
  {
    // Class instantiation
    MaintenanceStage maintenance;
    // Stage running
    maintenance.RunStage();
  }

  return(0);
}