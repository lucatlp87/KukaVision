// <MainLoop> CLASS METHODS IMPLEMENTATION

#include <iostream>
#include <string>

#include <boost/lexical_cast.hpp>

#include <Eigen/StdVector>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "MainLoop.h"
#include "Cloud.h"
#include "KinectAcquisition.h"
#include "CloudProcessing.h"
#include "OURCVFHEstimation.h"
#include "DBManagement.h"
#include "SceneManagement.h"

// Definition of the reference pair <model_path, object_idx>
typedef std::pair<std::string, int> ref_pair;

// CONSTRUCTOR
MainLoop::MainLoop() {}

// DESTRUCTOR
MainLoop::~MainLoop() {}

// #################################################################################################################################################################
// HELP ############################################################################################################################################################
// #################################################################################################################################################################

void 
MainLoop::PrintUsage(const char* progName)
{
	std::cout << std::endl << std::endl << "Usage: "<<progName<<" [options]" << std::endl << std::endl
    	      << "Options:" << std::endl
           	  << "-------------------------------------------" << std::endl
              << "-h            THIS HELP" << std::endl
              << "-training     TRAINING STAGE" << std::endl
              << "              Object model creation (Kinect acquisition of the cluster, VFH descriptor determination" << std::endl
              << "              and KdTree update)" << std::endl
              << "-newscene     NEW SCENE REGISTRATION STAGE" << std::endl
              << "              New reference scene acquisition" << std::endl
              << std::endl;
}


// #################################################################################################################################################################
// TRAINING STAGE IMPLEMENTATION ###################################################################################################################################
// #################################################################################################################################################################

void MainLoop::TrainingStage()
{
	  pcl::console::print_error("\n\n********************************************************************************\n");
    pcl::console::print_error("******************** TRAINING STAGE - OBJECT MODELS CREATION *******************\n");
    pcl::console::print_error("********************************************************************************\n");
    std::cout << "Please, place only one object in the center of the scene and repeat the TRAINING" << std::endl
              << "STAGE until you acquired a cloud from every point of view" << std::endl << std::endl;
    
    // INITIALIZATION
    // <Cloud> class object definition
    Cloud cloud_by_kinect;
    // Time counter
    pcl::console::TicToc tt;

    // New acquisition user choice
    std::string insert_object;
    bool first_time = 1;
    
    while(true)
    {
      // USER CHOICE PARSING ***************************************************************************************************************************************
      // In order to have the chance to do multiple acquisition (different point of view of the object) a <while> loop is performed and the user is asked to choose
      // between doing a new acquisition or exiting the function. In add, the flag <first_time> allows to separate the first access to the function (the first 
      // acquisition) that is enabled by default. After the first acquisition the user is asked to choose between exit the function or acquire another cloud.
      // The available options for the user to insert are:
      //  - <y> to acquire a new image (default for the first access)
      //  - <n> to exit the function
      //  If the user inserts a wrong letter an error message will be shown and the user will be asked to insert a new choice

      if (!first_time)
      {
        std::cout << std::endl << "Do you want to repeat the acquisition? " << std::flush;
        std::cin >> insert_object;
      }
      else 
      {
        insert_object = "y";
        first_time = 0;
      }    

      // NEW ACQUISITION *******************************************************************************************************************************************
      if (insert_object.compare ("y") == 0)
      {
        // Vector of updates subfolders of the DB
        std::vector<std::string> object_subfolders;

        // STEP T1. ACQUISITION OF THE CLOUD FROM KINECT ***********************************************************************************************************
        // *********************************************************************************************************************************************************
        // Be sure to have:
        //  - the Kinect sensor plugged in
        //  - the object of which you want to create the model in the center of the scene with no other object
        
        tt.tic();
        pcl::console::print_error ("\nSTEP T1. POINT CLOUD ACQUISITION FROM KINECT\n");
        
        // Acquisition class instantiation
        KinectAcquisition acquisition_object;

        // acquisition_object.AcquireCloudKinect(cloud_by_kinect);
        cloud_by_kinect.LoadCloud("scene.pcd");

        std::cout << std::endl <<  "---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
        
        // Save the original cloud in order to visualize it as background in clusters visualization
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
        original_acquisition = cloud_by_kinect.GetCloud();


        // STEP T2. PASS-THROUGH FILTER APPLICATION *****************************************************************************************************************
        // **********************************************************************************************************************************************************

        tt.tic();
        pcl::console::print_error ("\nSTEP T2. PASS-THROUGH FILTER APPLICATION");

        // Processing class instantiation
        CloudProcessing processing_object;

        // Filter application
        // processing_object.PassThroughFilter(cloud_by_kinect);

        std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

        // STEP T3. DOMINANT PLANE AND CLUSTERS EXTRACTION **********************************************************************************************************
        // **********************************************************************************************************************************************************

        // Vector containing the extracted clusters objects
        std::vector<Cloud, Eigen::aligned_allocator<Cloud> > extracted_clusters;
        
        tt.tic();
        pcl::console::print_error ("\nSTEP T3. DOMINANT PLANE AND CLUSTERS EXTRACTION\n");

        extracted_clusters = processing_object.ExtractClustersFromCloud(cloud_by_kinect);
        
        // Check if segmentation errors occurred or if no clusters were found
        if (!extracted_clusters[0].GetSegError() && !extracted_clusters[0].GetNoClusters())
        {
          std::vector<std::string> updated_folders (1,"NO_CLUSTER_SAVED");
          std::string win_legend_init = "TRAINING STAGE - CLUSTERS STORAGE\nCluster ";
          std::string win_legend_end = "/";
          win_legend_end.append(boost::lexical_cast<std::string>((int)extracted_clusters.size()));
          bool first_storage = 1;

          // STEP T3.1. CLUSTER VISUALIZATION AND STORAGE ********************************************************************************************************
          // *****************************************************************************************************************************************************
          // The next step is implement for each found cluster. It deals with visualization of the cluster (wrt the orignal acquired cloud) and the user choice 
          // about saving it or not.
          pcl::console::print_error ("\n\tSTEP T3.1. CLUSTERS VISUALIZATION AND STORAGE");

          for (int cluster_idx = 0; cluster_idx < extracted_clusters.size(); ++cluster_idx)
          {
            // Cloud pointer
            pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            vis_cloud = extracted_clusters[cluster_idx].GetCloud();
            std::string win_legend = win_legend_init;
            win_legend.append(boost::lexical_cast<std::string>(cluster_idx+1)).append(win_legend_end);

            // Visualization object
            boost::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer (new pcl::visualization::PCLVisualizer ("Cluster Viewer"));         
            cluster_viewer->initCameraParameters ();
            cluster_viewer->addCoordinateSystem (1.0);
            cluster_viewer->setBackgroundColor (0, 0, 0);
            cluster_viewer->setWindowBorders(true);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue (original_acquisition, 0, 0, 255);

            // Updated folder
            std::string updated_dir;
            // User choice
            std::string save_cloud;

            pcl::console::print_error("\n\tCLUSTER - %d (%d data points)\n", cluster_idx+1, extracted_clusters[cluster_idx].GetCloud()->points.size());
            
            // VISUALIZING THE CLOUD
            std::cout << " \tCluster visualization " << std::flush;
            pcl::console::print_error ("\t(Please close the visualizer to continue)\n");
              
            cluster_viewer->addPointCloud<pcl::PointXYZ> (original_acquisition, color_blue, "Original");
            cluster_viewer->addPointCloud<pcl::PointXYZ> (vis_cloud, "Cloud");
            cluster_viewer->resetCameraViewpoint("Original");
            cluster_viewer->resetCameraViewpoint("Cloud");
            cluster_viewer->addText(win_legend, 10, 10, 1, 0, 0, "AcquiredCloudText");
            cluster_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText");
            cluster_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud");

            while (!cluster_viewer->wasStopped ())
            { 
                cluster_viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (1000));
            }
            cluster_viewer->close();

            // User choice parsing
            std::cout << "\tDo you want to save this cloud? ";
            std::cin >> save_cloud;

            // If the user decides not to save the cloud, the first element of the return vector is settet to the value "NOT_SAVED" and then the loop considers the
            // next cluster.
            if (save_cloud.compare("y"))
              updated_dir = "NOT_SAVED";
            else
            {
              // STEP T3.1.1. VOXEL GRID FILTER APPLICATION ********************************************************************************************************
              // ***************************************************************************************************************************************************
              // In this step a Voxel grid filter is applied in order to uniform the data points density
              tt.tic();
              pcl::console::print_error("\n\t\tSTEP T3.1.1. Voxel grid filter application...");
                            
              processing_object.VoxelGridFilter(&extracted_clusters[cluster_idx]);

              pcl::console::print_error ("\tdone\n");
              std::cout << "\t\t(" << extracted_clusters[cluster_idx].GetCloud()->points.size() << " data points)" << std::endl
                        << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

              // STEP T3.1.2. NORMALS ESTIMATION *******************************************************************************************************************
              // ***************************************************************************************************************************************************
              // This step deals with normals computation and concatenation between point cloud and normals cloud.

              tt.tic();
              pcl::console::print_error ("\n\t\tSTEP T3.1.2. Normals computation...");

              processing_object.CloudNormalsComputation(&extracted_clusters[cluster_idx]);

              pcl::console::print_error ("\tdone\n");
              std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;
  
              // STEP T3.1.3. OUR-CVFH SIGNATURE ESTIMATION ********************************************************************************************************
              // ***************************************************************************************************************************************************

              // OUR-CVFH estimation class instantiation
              OURCVFHEstimation ourcvfh_object;

              tt.tic();
              pcl::console::print_error ("\n\t\tSTEP T3.1.3 OUR-CVFH signature estimation...");

              // ourcvfh_object.CloudOURCVFHComputation(&extracted_clusters[cluster_idx]);
              if (cluster_idx == 0)
                extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/milk/milk_front_ourcvfh.pcd");
              else if(cluster_idx == 1)
                extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_sx_ourcvfh.pcd");
              else if(cluster_idx == 2)
                extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_dx_ourcvfh.pcd");


              pcl::console::print_error ("\tdone\n");
              std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

              // STEP T3.1.4 SAVING THE CLOUD ***********************************************************************************************************************
              // ****************************************************************************************************************************************************
              pcl::console::print_error ("\n\t\tSTEP T3.1.4. Cloud storage...\n");

              updated_dir = extracted_clusters[cluster_idx].SaveCloud();

              // Update the return vector
              if (first_storage)
              {
                updated_folders.assign(1, updated_dir);
                first_storage = 0;
              }
              else
              {
                bool insert_ok = 0;
                for (int ret_idx = 0; ret_idx < updated_folders.size(); ++ret_idx)
                  if (updated_folders[ret_idx].compare(updated_dir) == 0)
                    insert_ok = 1;
            
                if (insert_ok == 0)
                  updated_folders.push_back(updated_dir);
              }
            }            
          }

          std::cout << std::endl << "---> DOMINANT PLANE AND CLUSTERS EXTRACTION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
               
          // STEP T4. KDTREE UPDATE *********************************************************************************************************************************
          // ********************************************************************************************************************************************************

          tt.tic();
          pcl::console::print_error ("\nSTEP T4. KD-TREES UPDATE\n");
          
          // Prcessing the modified subfolder vector. 
          if (!updated_folders[0].compare("NO_CLUSTER_SAVED"))
            // If the vector <updated_folders> is empty (the first element is equal to NO_CLUSTERS_SAVED) this step is skipped and a warining message is printed out.
            pcl::console::print_error("\n\t[WARNING] No new clusters added! KdTrees are up to date!\n");
          else
          {
            // Time counter
            pcl::console::TicToc tt_update;

            std::cout << "(" << updated_folders.size() << " modified subfolder(s))" << std::endl;

            // The update procedure is performed for each modified DB subfolder.
            for (int loop_idx = 0; loop_idx < updated_folders.size(); ++loop_idx)
            {
              // Update class instantiation
              DBManagement update_object;
              
              std::cout << std::endl << "UPDATING THE FOLDER " << updated_folders[loop_idx] << std::flush;

              // Update all paths and search the actual subfolder for .pcd files.
              update_object.UpdateObject(updated_folders[loop_idx]);
              std::cout << "\t(" << update_object.GetFoundSignatureNumber() << " histograms found)" << std::endl;
              
              // STEP T4.1 OUR-CVFH HISTOGRAM CONVERSION TO FLANN FORMAT *********************************************************************************************
              // *****************************************************************************************************************************************************
              // The first step of the update procedure deals with the conversion of OUR-CVFH histogram point cloud into a Flann matrix. Once the matrix is created 
              // and filled it is saved in the current subfolder of the DB.
              tt_update.tic();
              pcl::console::print_error ("\tSTEP T4.1. OUR-CVFH histrogram conversion to Flann format...");

              // Flann conversion
              update_object.OURCVFHFlannConversion(updated_folders[loop_idx]);

              pcl::console::print_error ("\tdone\n");
              std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

              // STEP T4.2 OUR-CVFH HISTOGRAM FILE NAME STORAGE ******************************************************************************************************
              // *****************************************************************************************************************************************************
              // The second step consist in adding (in append mode) the new histrogrms paths to the file containing names of hall istograms of the current subfolder.

              tt_update.tic();
              pcl::console::print_error ("\n\tSTEP T4.2. OUR-CVFH histograms path add...");

              // Name list update
              update_object.HistogramNameStorage(updated_folders[loop_idx]);

              pcl::console::print_error ("\tdone\n");
              std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

              // STEP T4.3 CREATION, BUILD AND STORAGE OF THE KD-TREE ***********************************************************************************************
              // ****************************************************************************************************************************************************
              // The last step consists in creating and build a tree index about all elements of the current subfolder.

              tt_update.tic();
              pcl::console::print_error ("\n\tSTEP T4.3. Kd-Tree index build and storage...");

              // Index building and storage
              update_object.BuildOURCVFHTreeIndex(updated_folders[loop_idx]);

              pcl::console::print_error ("\tdone\n");
              std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;
            }
          }
      
         std::cout << std::endl << "---> KD-TREES UPDATE total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
        }
      }
      else if (insert_object.compare("n") == 0)
        return;        
      else
        // Print an error message due to a wrong input
        pcl::console::print_error ("\n\t\t[COMMAND ERROR] Please insert <y> or <n>\n");
    }
}


// #################################################################################################################################################################
// NEW REFERENCE SCENE CREATION IMPLEMENTATION #####################################################################################################################
// #################################################################################################################################################################

void 
MainLoop::NewSceneStage()
{
	pcl::console::print_error("\n\n******************************************************\n");
  pcl::console::print_error("NEW REFERENCE SCENE STAGE - SCENE CREATION AND STORAGE\n");
  pcl::console::print_error("******************************************************\n");
  std::cout << "Please set the scene and start this procedure in order to register a new reference scene" 
            << std::endl << std::endl;

  // INITIALIZATION
  // <Cloud> class object definition
  Cloud reference_scene;
  // <SceneManagement> object instantiation
  SceneManagement scene_object;
  // Current object id
  int current_obj_id = 1;

  // Time counter
  pcl::console::TicToc tt;

  // Auxiliary matrix (to be canceled)
  flann::Matrix<float> mm (new float[16],4,4);
            for (size_t ii=0;ii<4;++ii)
              for (size_t jj=0;jj<4;++jj)
                mm[ii][jj]=1;

  // STEP NS1. ACQUISITION OF THE CLOUD FROM KINECT ***********************************************************************************************************
  // *********************************************************************************************************************************************************
  // Be sure to have:
  //  - the Kinect sensor plugged in
  //  - the new reference scene you wanto to acquire in the visible range of the Kinect
        
  tt.tic();
  pcl::console::print_error ("\nSTEP NS1. POINT CLOUD ACQUISITION FROM KINECT\n");
       
  // Acquisition class instantiation
  KinectAcquisition acquisition_object;

  // acquisition_object.AcquireCloudKinect(reference_scene);
  reference_scene.LoadCloud("scene.pcd");

  // Saving the cloud in the <SceneManagement> object
  scene_object.SetRefScene(reference_scene.GetCloud());

  std::cout << std::endl <<  "\t---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
        
  // Save the original cloud in order to visualize it as background in clusters visualization
  // pcl::PointCloud<pcl::PointXYZ>::Ptr original_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
  // original_acquisition = reference_scene.GetCloud();

  // STEP NS2. PASS-THROUGH FILTER APPLICATION **********************************************************************************************************
  // **********************************************************************************************************************************************************

  tt.tic();
  pcl::console::print_error ("\nSTEP NS2. PASS-THROUGH FILTER APPLICATION");

  // Processing class instantiation
  CloudProcessing processing_object;

  // Filter application
  // processing_object.PassThroughFilter(reference_scene);

  std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;


  // STEP NS3. DOMINANT PLANE AND CLUSTERS EXTRACTION **********************************************************************************************************
  // **********************************************************************************************************************************************************

  // Vector containing the extracted clusters objects
  std::vector<Cloud, Eigen::aligned_allocator<Cloud> > extracted_clusters;
        
  tt.tic();
  pcl::console::print_error ("\nSTEP NS3. DOMINANT PLANE AND CLUSTERS EXTRACTION\n");

  extracted_clusters = processing_object.ExtractClustersFromCloud(reference_scene);
   
  // Check if segmentation errors occurred or if no clusters were found
  if (!extracted_clusters[0].GetSegError() && !extracted_clusters[0].GetNoClusters())
  {
    std::vector<std::string> updated_folders (1,"NO_CLUSTER_SAVED");
    bool first_storage = 1;
      
    // STEP NS4. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH *****************************************************************************************
    // ***********************************************************************************************************************************************************
    // The next step is implement for each found cluster. It deals with the determination of OUR-CVFH signature and woth the search of that signture in the DB.
    pcl::console::print_error ("\nSTEP NS4. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH");

    for (int cluster_idx = 0; cluster_idx < extracted_clusters.size(); ++cluster_idx)
    {

      // STEP NS4.1. VOXEL GRID FILTER APPLICATION ***************************************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step a Voxel grid filter is applied in order to uniform the data points density
      
      // Path of the subfolder
      std::string search_path;
      // Temporary <Object> instantiation
      Object current_object;

      // Flag indicating correspondence found
      bool corr_found = 0;

      tt.tic();
      pcl::console::print_error("\n\tSTEP NS4.1. VOXEL GRID FILTER APPLICATION...");
                            
      processing_object.VoxelGridFilter(&extracted_clusters[cluster_idx]);

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t(" << extracted_clusters[cluster_idx].GetCloud()->points.size() << " data points)" << std::endl
                << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

      // STEP NS4.2. NORMALS ESTIMATION **************************************************************************************************************************
      // *********************************************************************************************************************************************************
      // This step deals with normals computation and concatenation between point cloud and normals cloud.

      tt.tic();
      pcl::console::print_error ("\n\tSTEP NS4.2. NORMALS COMPUTATION...");

      processing_object.CloudNormalsComputation(&extracted_clusters[cluster_idx]);

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;
  
      // STEP NS4.3. OUR-CVFH SIGNATURE ESTIMATION ***************************************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step the OUR-CVFH signature is calculated 

      // OUR-CVFH estimation class instantiation
      OURCVFHEstimation ourcvfh_object;

      tt.tic();
      pcl::console::print_error ("\n\tSTEP NS4.3. OUR-CVFH SIGNATURE ESTIMATION...");

      // ourcvfh_object.CloudOURCVFHComputation(&extracted_clusters[cluster_idx]);
      if (cluster_idx == 0)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/milk/milk_front_ourcvfh.pcd");
      else if(cluster_idx == 1)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_sx_ourcvfh.pcd");
      else if(cluster_idx == 2)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_dx_ourcvfh.pcd");

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

      // STEP NS4.3.1 OUR-CVFH SIGNATURE TRANSFORMATION TO FLOAT VECTOR ******************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step the OUR-CVFH signature is tranformed in a float vector (attribute of the class Cloud)

      pcl::console::print_error ("\n\t\tSTEP NS4.3.1 OUR-CVFH signature transformation...");

      extracted_clusters[cluster_idx].SetOURCVFHVector();

      pcl::console::print_error ("\tdone\n");

      // STEP NS4.4 DB SEARCH ************************************************************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step the main folder of the DB is searched in order to find a correspondence with the extracted cluster histogram. Each sub-folder is explored 
      // searching the correspondent Kd-Tree.

      // DBManagement object instantiation
      DBManagement search_object;
      // DB base dir
      boost::filesystem::path DB_base_dir = "../ObjectDB/";

      tt.tic();
      pcl::console::print_error ("\n\tSTEP NS4.4. DB SEARCH...");

      if (!boost::filesystem::exists (DB_base_dir) && !boost::filesystem::is_directory (DB_base_dir))
      {
        pcl::console::print_error("\n   [FATAL ERROR] No DataBase found!\n");
        return;
      }

      for (boost::filesystem::directory_iterator it (DB_base_dir); it != boost::filesystem::directory_iterator (); ++it)
      {
        if (boost::filesystem::is_directory (it->status ()))
        {
          std::stringstream ss;
          ss << it->path ().c_str();
          search_path = ss.str();
          search_path.append("/");
          std::cout << "\n\tSearching the folder " << search_path << " ... " << std::flush;

          // Updating search object attributes
          search_object.UpdateObject(search_path);

          // Searching the folder and updating the path vector
          if (search_object.SearchTheDB(extracted_clusters[cluster_idx]))
          {
            // Update the object list
            current_object.SetObjectID(current_obj_id);
            ++current_obj_id;
            current_object.SetObjectType(ss.str().substr(ss.str().find_last_of("/")+1));
            current_object.SetObjectModelPath(search_path);
            current_object.SetObjectRefPose(mm);

            scene_object.InsertObject(current_object);

            corr_found = 1;
            break;
          }
        }
      }
      if (!corr_found)
      {
        // Visualization cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        vis_cloud = extracted_clusters[cluster_idx].GetCloud();

        // User choice
        std::string save_new_model;

        // Visualization object
        boost::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer (new pcl::visualization::PCLVisualizer ("Cluster Viewer"));         
        cluster_viewer->initCameraParameters ();
        cluster_viewer->addCoordinateSystem (1.0);
        cluster_viewer->setBackgroundColor (0, 0, 0);
        cluster_viewer->setWindowBorders(true);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue (scene_object.GetRefScene(), 0, 0, 255);

        pcl::console::print_error("\n\t[WARNING] The considered cluster has no correspondences in the DB!\n");
        pcl::console::print_error("\tPlease close the visualizer to continue...\n");

        cluster_viewer->addPointCloud<pcl::PointXYZ> (scene_object.GetRefScene(),color_blue,"Original");
        cluster_viewer->addPointCloud<pcl::PointXYZ> (vis_cloud, "Cloud");
        cluster_viewer->resetCameraViewpoint("Original");
        cluster_viewer->resetCameraViewpoint("Cloud");
        cluster_viewer->addText("NEW SCENE STAGE - NEW OBJECT FOUND\nObject model storage", 10, 10, 1, 0, 0, "AcquiredCloudText");
        cluster_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText");
        cluster_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud");



        while (!cluster_viewer->wasStopped ())
        { 
            cluster_viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (1000));
        }
        cluster_viewer->close();

        std::cout << "\tDo you want to save it as a new model? " << std::flush;
        std::cin >> save_new_model;

        if(!save_new_model.compare("y"))
        {
          // Update object
          DBManagement new_model_update;
          // Time counter
          pcl::console::TicToc tt_update;

          // SAVING THE CLOUD **************************************************************************************************************************************
          pcl::console::print_error ("\n\tSaving the cloud...\n");
          search_path = extracted_clusters[cluster_idx].SaveCloud();


          // UPDATING THE FOLDER ***********************************************************************************************************************************
          std::cout << std::endl << "\tUPDATING THE FOLDER " << search_path << std::flush;

          // Update all paths and search the actual subfolder for .pcd files.
          new_model_update.UpdateObject(search_path);
          std::cout << "\t(" << search_object.GetFoundSignatureNumber() << " histograms found)" << std::endl;
              
          // STEP NS4.4.1. OUR-CVFH HISTOGRAM CONVERSION TO FLANN FORMAT *********************************************************************************************
          // *****************************************************************************************************************************************************
          // The first step of the update procedure deals with the conversion of OUR-CVFH histogram point cloud into a Flann matrix. Once the matrix is created 
          // and filled it is saved in the current subfolder of the DB.
          tt_update.tic();
          pcl::console::print_error ("\tSTEP NS4.4.1. OUR-CVFH histrogram conversion to Flann format...");

          // Flann conversion
          new_model_update.OURCVFHFlannConversion(search_path);

          pcl::console::print_error ("\tdone\n");
          std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

          // STEP NS3.4.2. OUR-CVFH HISTOGRAM FILE NAME STORAGE ******************************************************************************************************
          // *****************************************************************************************************************************************************
          // The second step consist in adding (in append mode) the new histrogrms paths to the file containing names of hall istograms of the current subfolder.

          tt_update.tic();
          pcl::console::print_error ("\n\tSTEP NS4.4.2. OUR-CVFH histograms path add...");

          // Name list update
          new_model_update.HistogramNameStorage(search_path);

          pcl::console::print_error ("\tdone\n");
          std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

          // STEP NS3.4.3. CREATION, BUILD AND STORAGE OF THE KD-TREE ***********************************************************************************************
          // ****************************************************************************************************************************************************
          // The last step consists in creating and build a tree index about all elements of the current subfolder.

          tt_update.tic();
          pcl::console::print_error ("\n\tSTEP NS4.4.3. Kd-Tree index build and storage...");

          // Index building and storage
          new_model_update.BuildOURCVFHTreeIndex(search_path);

          pcl::console::print_error ("\tdone\n");
          std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;
       
          // Update the object list
          current_object.SetObjectID(current_obj_id);
          ++current_obj_id;
          current_object.SetObjectModelPath(search_path);
          search_path.erase(search_path.end()-1);
          current_object.SetObjectType(search_path.substr(search_path.find_last_of("/")+1));
          current_object.SetObjectRefPose(mm);

          scene_object.InsertObject(current_object);        
        }
        else
          pcl::console::print_error("\n\t[WARNING] The considered cluster will not be part of the reference scene!\n");
      }
      std::cout << std::endl <<  "\t---> DB SEARCH total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
    }
  }

  // STEP NS5. SAVING OBJECTS INFOS *********************************************************************************************************************************
  // ***************************************************************************************************************************************************************
  // In this section the vector <correspondance_folder_list> is saved as ObjectsList.list. 
  tt.tic();
  pcl::console::print_error ("STEP NS5. SCENE STORAGE\n");

  // Saving the scene
  scene_object.SaveScene();

  std::cout << std::endl <<  "---> SCENE STORAGE total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

  // STEP NS6. VISUALIZING THE SCENE ********************************************************************************************************************************
  // ***************************************************************************************************************************************************************
  pcl::console::print_error ("STEP NS6. SCENE VISUALIZATION\n");
  pcl::console::print_error ("Please quit the visualization to continue\n\n");

  scene_object.VisualizeRefScene(extracted_clusters);

}

// #################################################################################################################################################################
// KUKAVISION IMPELEMENTATION ######################################################################################################################################
// #################################################################################################################################################################

void 
MainLoop::KukaVisionStage()
{
	pcl::console::print_error("\n\n******************************************************************\n");
  pcl::console::print_error("KUKAVISION STAGE - REGISTER A MODIFIED SCENE AND PLAN TO-DO ACTION\n");
  pcl::console::print_error("******************************************************************\n");
  std::cout << "Please set the scene and start this procedure in order to register a new scene and calculate to-do action to keep the scene equal to the reference one" 
            << std::endl << std::endl;

  // INITIALIZATION

  // <Cloud> class object definition
  Cloud modified_scene;
  // <SceneManagement> object instantiation for the actual modified scene
  SceneManagement actual_scene_object;
  // Vector containing all ObjectDB subfolders of the reference scene
  std::vector<ref_pair> ref_subfolders;

  // Time counter
  pcl::console::TicToc tt;

  // STEP KV1. SELECTION OF THE REFERENCE SCENE *******************************************************************************************************************
  // ***************************************************************************************************************************************************************
  // The preliminar step consists in asking the user to specify the reference scene with which compare the actual scene acquired by kinect.

  // String containing the user choice
  std::string chosen_ref_scene;
  // String containing user choice about a reselection of sscene
  std::string another_scene;

  pcl::console::print_error ("\nSTEP KV1. REFERENCE SCENE SELECTION\n");
  
  while (true)
  {
    // Path to the base directory of the scenes DB
    std::string path_scene_DB = "../SceneDB/";

    std::cout << "\tPlease insert the name of the reference scene you want to consider: " << std::flush;
    std::cin >> chosen_ref_scene;

    // Update the path to the chosen scene
    path_scene_DB.append(chosen_ref_scene);
    path_scene_DB.append("/");

    if (!boost::filesystem::exists (path_scene_DB) && !boost::filesystem::is_directory (path_scene_DB))
    {
      pcl::console::print_error("\n\t[ERROR] The DB does not contain any scene corresponding to the specified name!\n");
      std::cout << "\tDo you want to specify another scene name? (if no the function will terminate) " << std::flush;
      std::cin >> another_scene;

      if (!another_scene.compare("n"))
        return;
    }
    else
    {
      tt.tic();
      pcl::console::print_error ("\tUploading the reference scene...");

      // Upload the scene object
      actual_scene_object.LoadRefScene(chosen_ref_scene);
      // Upload the list of model subfolders
      ref_subfolders = actual_scene_object.GetModelSubfolderList();
      
      pcl::console::print_error ("\tdone\n");
      std::cout << "\t---> REFERENCE SCENE DEFINITION total execution time: " << tt.toc() << " ms" << std::endl;
      break; 
    }
  }


  // STEP KV2. ACQUISITION OF THE CLOUD FROM KINECT ****************************************************************************************************************
  // ***************************************************************************************************************************************************************
  // Be sure to have:
  //  - the Kinect sensor plugged in
  //  - the new reference scene you wanto to acquire in the visible range of the Kinect
        
  tt.tic();
  pcl::console::print_error ("\nSTEP KV2. POINT CLOUD ACQUISITION FROM KINECT\n");
       
  // Acquisition class instantiation
  KinectAcquisition acquisition_object;

  // acquisition_object.AcquireCloudKinect(modified_scene);
  modified_scene.LoadCloud("scene.pcd");

  // Saving the cloud in the <SceneManagement> object
  actual_scene_object.SetRefScene(modified_scene.GetCloud());

  std::cout << std::endl <<  "\t---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
        
  // Save the original cloud in order to visualize it as background in clusters visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
  original_acquisition = modified_scene.GetCloud();

  // STEP NS2. PASS-THROUGH FILTER APPLICATION ****************************************************************************************************************
  // **********************************************************************************************************************************************************

  tt.tic();
  pcl::console::print_error ("\nSTEP KV3. PASS-THROUGH FILTER APPLICATION");

  // Processing class instantiation
  CloudProcessing processing_object;

  // Filter application
  // processing_object.PassThroughFilter(modified_scene);

  std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

  // STEP KV4. DOMINANT PLANE AND CLUSTERS EXTRACTION **********************************************************************************************************
  // **********************************************************************************************************************************************************

  // Vector containing the extracted clusters objects
  std::vector<Cloud, Eigen::aligned_allocator<Cloud> > extracted_clusters;
        
  tt.tic();
  pcl::console::print_error ("\nSTEP KV4. DOMINANT PLANE AND CLUSTERS EXTRACTION\n");

  extracted_clusters = processing_object.ExtractClustersFromCloud(modified_scene);
   
  // Check if segmentation errors occurred or if no clusters were found
  if (!extracted_clusters[0].GetSegError() && !extracted_clusters[0].GetNoClusters())
  {
    std::vector<std::string> updated_folders (1,"NO_CLUSTER_SAVED");
      
    // STEP KV5. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH *****************************************************************************************
    // ***********************************************************************************************************************************************************
    // The next step is implement for each found cluster. It deals with the determination of OUR-CVFH signature and woth the search of that signture in the DB.
    pcl::console::print_error ("\nSTEP KV5. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH\n");

    for (int cluster_idx = 0; cluster_idx < extracted_clusters.size(); ++cluster_idx)
    {
      std::cout << "\n\tCLUSTER N.  " << cluster_idx+1 << "--------------------------------------------------------------------------" << std::flush;

      // STEP KV4.1. VOXEL GRID FILTER APPLICATION ***************************************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step a Voxel grid filter is applied in order to uniform the data points density
      
      // Path of the subfolder
      std::string search_path;

      // Flag indicating correspondence found
      bool corr_found = 0;

      tt.tic();
      pcl::console::print_error("\n\tSTEP KV5.1. VOXEL GRID FILTER APPLICATION...");
                            
      processing_object.VoxelGridFilter(&extracted_clusters[cluster_idx]);

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t(" << extracted_clusters[cluster_idx].GetCloud()->points.size() << " data points)" << std::endl
                << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

      // STEP KV5.2. NORMALS ESTIMATION **************************************************************************************************************************
      // *********************************************************************************************************************************************************
      // This step deals with normals computation and concatenation between point cloud and normals cloud.

      tt.tic();
      pcl::console::print_error ("\n\tSTEP KV5.2. NORMALS COMPUTATION...");

      processing_object.CloudNormalsComputation(&extracted_clusters[cluster_idx]);

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;
  
      // STEP KV5.3. OUR-CVFH SIGNATURE ESTIMATION ***************************************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step the OUR-CVFH signature is calculated 

      // OUR-CVFH estimation class instantiation
      OURCVFHEstimation ourcvfh_object;

      tt.tic();
      pcl::console::print_error ("\n\tSTEP KV5.3. OUR-CVFH SIGNATURE ESTIMATION...");

      // ourcvfh_object.CloudOURCVFHComputation(&extracted_clusters[cluster_idx]);
      if (cluster_idx == 0)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/milk/milk_front_ourcvfh.pcd");
      else if(cluster_idx == 1)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_sx_ourcvfh.pcd");
      else if(cluster_idx == 2)
          extracted_clusters[cluster_idx].LoadOURCVFH("../ObjectDB_old/detergent/detergent_dx_ourcvfh.pcd");

      pcl::console::print_error ("\tdone\n");
      std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

      // STEP KV5.3.1 OUR-CVFH SIGNATURE TRANSFORMATION TO FLOAT VECTOR ******************************************************************************************
      // *********************************************************************************************************************************************************
      // In this step the OUR-CVFH signature is tranformed in a float vector (attribute of the class Cloud)

      pcl::console::print_error ("\n\t\tSTEP KV5.3.1 OUR-CVFH signature transformation...");

      extracted_clusters[cluster_idx].SetOURCVFHVector();

      pcl::console::print_error ("\tdone\n");

      // STEP KV5.4 DB SEARCH ************************************************************************************************************************************
      // *********************************************************************************************************************************************************
      // The ObjectDB search is performed checking only folders in the variable <subfolder_list> of the object <ref_scene_management>

      // DBManagement object instantiation
      DBManagement search_object;
      // DB base dir
      boost::filesystem::path DB_base_dir = "../SceneDB/";

      tt.tic();
      pcl::console::print_error ("\n\tSTEP KV5.4. DB SEARCH...");

      if (!boost::filesystem::exists (DB_base_dir) && !boost::filesystem::is_directory (DB_base_dir))
      {
        pcl::console::print_error("\n   [FATAL ERROR] No DataBase found!\n");
        return;
      }

      if (ref_subfolders.size() > 0)
      {
        for (int sub_idx = 0; sub_idx < ref_subfolders.size(); ++sub_idx)
        {
          std::cout << "\n\tSearching the folder " << ref_subfolders[sub_idx].first << " ... " << std::flush;

          if (!boost::filesystem::exists (ref_subfolders[sub_idx].first) && !boost::filesystem::is_directory (ref_subfolders[sub_idx].first))
          {
            pcl::console::print_error ("\n[ERROR] The ObjectDB folder you are considering does not exist!\n\n");
            return;
          }

          // Updating search object attributes
          search_object.UpdateObject(ref_subfolders[sub_idx].first);

          // Searching the folder and updating the path vector
          if (search_object.SearchTheDB(extracted_clusters[cluster_idx]))
          {
            // Erase the element of the subfolder_list vector corresponding to the found cluster
            actual_scene_object.UpdateInnerObjectsVector(ref_subfolders[sub_idx].second);
            ref_subfolders.erase(ref_subfolders.begin()+sub_idx);
            corr_found = 1;
            break;
          }
        }

        if (corr_found == 0)
        {
          std::cout << std::endl << "\tThe current cluster is not part of the reference scene!" 
                    << std::endl << "\tIt has to eliminated from the scene!" << std::endl;

          actual_scene_object.UpdateOuterObjectsVector(extracted_clusters[cluster_idx], cluster_idx);
        }
      }
    }

    pcl::console::print_error ("\n\tResume\n");
    std::cout << "\t" << actual_scene_object.GetInObjectsVector().size() << " objects with correspondence found" << std::endl
              << "\t" << actual_scene_object.GetOutObjectsVector().size() << " objects with no correspondence found" << std::endl;
  }
}

// #################################################################################################################################################################
// SIMPLE VISUALIZATION OF A CLOUD #################################################################################################################################
// #################################################################################################################################################################

void 
MainLoop::SimpleVisualizer()
{
  SceneManagement scene;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("OneObjectScene.pcd",*cloud_ptr);

  scene.SetRefScene(cloud_ptr);
  // scene.VisualizeRefScene();
}