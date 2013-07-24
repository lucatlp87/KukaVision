// <KukaVisionStage> CLASS METHODS DEFINITION
#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "KukaVisionStage.h"
#include "KinectAcquisition.h"
#include "SceneManagement.h"
#include "CloudProcessing.h"
#include "OURCVFHEstimation.h"
#include "DBManagement.h"

// Definition of the reference pair <model_path, object_idx>
typedef std::pair<std::string, int> ref_pair;

// CONSTRUCTOR
KukaVisionStage::KukaVisionStage() {}

// DESTRUCTOR
KukaVisionStage::~KukaVisionStage() {}

// #################################################################################################################################################################
// KUKAVISION STAGE IMPLEMENTATION #################################################################################################################################
// #################################################################################################################################################################

void
KukaVisionStage::RunStage()
{
	  pcl::console::print_error("\n\n#######################################################################################");
	  pcl::console::print_error("#######################################################################################\n");
	  pcl::console::print_error("################################################################## KUKAVISION STAGE - OBJECT MODELS CREATION");
	  pcl::console::print_error(" #################################################################\n");
    pcl::console::print_error("#######################################################################################");
	  pcl::console::print_error("#######################################################################################\n");
	  std::cout << "Please set the scene and start this procedure in order to register a new scene and calculate to-do action to keep the scene equal to the reference one"  
			        << std::endl << std::endl;
 	
 	  // INITIALIZATION
    // Acquired cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
  	// Acquisition class instantiation
  	KinectAcquisition acquisition_object;
  	// <SceneManagement> object instantiation
  	SceneManagement scene_object;
    // Processing class instantiation
    CloudProcessing processing_object;
  	// Path to the base directory of the scenes DB
   	std::string base_path_scene_DB = "../SceneDB/";
   	// Path to the selected scene
   	std::string scene_path;
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

	  std::cout << "\tAvailable scenes in the DB are:" << std::endl << std::endl;
	  scene_object.ListDBItems();
	  std::cout << "\n\tPlease insert the name of the reference scene you want to consider: " << std::flush;
   	std::cin >> chosen_ref_scene;

   	while (true)
   	{

   		// Update the path to the chosen scene
   		scene_path = base_path_scene_DB;
   		scene_path.append(chosen_ref_scene);
   		scene_path.append("/");
    	
    	if (!boost::filesystem::exists (scene_path) && !boost::filesystem::is_directory (scene_path))
    	{
      		pcl::console::print_error("\n\t[WARNING] The DB does not contain any scene corresponding to the specified name!\n");
      		std::cout << "\tPlease specify another scene name (to exit please insert ""n"") " << std::flush;
      		std::cin >> chosen_ref_scene;

      		std::cout << chosen_ref_scene << std::endl;

      		if (!chosen_ref_scene.compare("n"))
        		return;
    	}
    	else
    		break;
    }

    // Loading the reference scene
    tt.tic();
    pcl::console::print_error ("\n\tUploading the reference scene...");

    // Upload the scene object
    if (!scene_object.LoadRefScene(chosen_ref_scene))
    {
      // Upload the list of model subfolders
      ref_subfolders = scene_object.GetModelSubfolderList();
      
      std::cout << "\n---> REFERENCE SCENE DEFINITION total execution time: " << tt.toc() << " ms" << std::endl;
    }
    else
      return;


    // STEP KV2. ACQUISITION OF THE CLOUD FROM KINECT ***********************************************************************************************************
  	// *********************************************************************************************************************************************************
	  // Be sure to have:
  	//  - the Kinect sensor plugged in
  	//  - the new reference scene you wanto to acquire in the visible range of the Kinect
        
  	tt.tic();
  	pcl::console::print_error ("\nSTEP KV2. POINT CLOUD ACQUISITION FROM KINECT ");
  	pcl::console::print_error("****************************************************************");
    pcl::console::print_error("****************************************************************\n");

  	// acquisition_object.AcquireCloudKinect(kinect_acquisition);
	  pcl::io::loadPCDFile("scene.pcd",*kinect_acquisition);

  	// Saving the cloud of the scene in the <SceneManagement> object
  	scene_object.SetActualScene(kinect_acquisition);

  	std::cout << std::endl <<  "---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;


  	// STEP KV3. PASS-THROUGH FILTER APPLICATION **********************************************************************************************************
  	// **********************************************************************************************************************************************************

  	tt.tic();
  	pcl::console::print_error ("\nSTEP KV3. PASS-THROUGH FILTER APPLICATION ");
  	pcl::console::print_error ("******************************************************************");
    pcl::console::print_error ("******************************************************************\n");

  	// Filter application
  	// processing_object.PassThroughFilter(kinect_acquisition);

  	std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;


    // STEP KV4. DOMINANT PLANE AND CLUSTERS EXTRACTION *****************************************************************************************************
    // *****************************************************************************************************************************************************
    tt.tic();
    pcl::console::print_error ("\nSTEP KV4. DOMINANT PLANE AND CLUSTERS EXTRACTION ");
    pcl::console::print_error ("***************************************************************");
    pcl::console::print_error ("**************************************************************\n");

    // INITIALIZATION OF VARIABLES
    // Segmented cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Cloud indices representing planar components inliers
    pcl::PointIndices::Ptr planar_inliers (new pcl::PointIndices);
    // Cloud coefficients for planar components inliers
    pcl::ModelCoefficients::Ptr planar_coefficients (new pcl::ModelCoefficients);
    // Segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> SAC_filter;
    // KdTree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr segmentation_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // Indices Extraction object
    pcl::ExtractIndices<pcl::PointXYZ> planar_inliers_extraction;
    // Euclidean Cluster Extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_exctraction;
    // Cluster indices vector
    std::vector<pcl::PointIndices> clusters_indices_vector;
    // Vector containing point clouds corresponding to objects cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters_cloud_vector;
    // Vector containing updated folders
    std::vector<std::string> updated_folders (1,"NO_CLUSTER_SAVED");

    // Time counter
    pcl::console::TicToc euclidean_tt;

    // Number of found clusters 
    int number_of_clusters;   
    // index
    int list_idx = 1;

    // PLANAR CLUSTER ELIMINATION **************************************************************************************************************************
    // *****************************************************************************************************************************************************
    // The first step consists in extracting the dominant plane cluster from the input cloud by using RANSAC algorithm and a predefined plane model. 

    euclidean_tt.tic();
    pcl::console::print_error ("\tDominant plane cluster extraction...");

    // Segmentation object initialization
    SAC_filter.setOptimizeCoefficients (true); 
    SAC_filter.setModelType(pcl::SACMODEL_PLANE);
    SAC_filter.setMethodType (pcl::SAC_RANSAC);
    SAC_filter.setMaxIterations (100);
    SAC_filter.setDistanceThreshold (0.02);

    // Segment the dominant plane cluster
    SAC_filter.setInputCloud (kinect_acquisition);
    SAC_filter.segment (*planar_inliers, *planar_coefficients);

    if (planar_inliers->indices.size () == 0)
    {
        pcl::console::print_error ("\n\t\t[SEGMENTATION ERROR] Could not estimate a plane model for the given dataset");  
        pcl::console::print_error ("\n\t\tPlease consider a new dataset or change the segmentation model and try again\n");
        return;
    }

    // Remove the planar cluster from the input cloud
    planar_inliers_extraction.setInputCloud (kinect_acquisition);
    planar_inliers_extraction.setIndices (planar_inliers);
    planar_inliers_extraction.setNegative (true);
    planar_inliers_extraction.filter (*no_plane_cloud);

    pcl::console::print_error ("  done\n");
    std::cout << "\tThe remaining cloud has " << no_plane_cloud->points.size() << " points" << std::endl
              << "\t<(execution time: " << euclidean_tt.toc() << " ms)>" << std::endl;

    // NaN ELEMENTS REMOVAL *********************************************************************************************************************************
    // ******************************************************************************************************************************************************
    // In order to apply the Euclidean Clusterign algorithm, NaN elements of the cloud have to be removed

    euclidean_tt.tic();
    pcl::console::print_error ("\n\tNaN elements removal...");

    std::vector<int> no_Nan_vector;
    pcl::removeNaNFromPointCloud(*no_plane_cloud,*no_plane_cloud,no_Nan_vector);
          
    pcl::console::print_error ("\tdone\n");
    std::cout << "\tThe remaining cloud has " << no_plane_cloud->points.size() << " points" << std::endl
            << "\t<(execution time: " << euclidean_tt.toc() << " ms)>" << std::endl;

    // EUCLIDEAN CLUSTER EXTRACTION *************************************************************************************************************************
    // ******************************************************************************************************************************************************
    // The second step consists in  extracting all clusters of the remaining cloud. Clusters have to respect certain properties to be valid.

    euclidean_tt.tic();
    pcl::console::print_error ("\n\tEuclidean cluster extraction...");

    // K-d tree initialization
    segmentation_tree->setInputCloud (no_plane_cloud);
    // Cluster extraction object initializations
    cluster_exctraction.setClusterTolerance (0.05);
    cluster_exctraction.setMinClusterSize (1000);
    cluster_exctraction.setSearchMethod (segmentation_tree);
    cluster_exctraction.setInputCloud (no_plane_cloud);

    // Cluster extraction
    // Each element of the vector <clusters_indices_vector> corresponds to a cluster of the cloud
    cluster_exctraction.extract (clusters_indices_vector);

    number_of_clusters = clusters_indices_vector.size();

    pcl::console::print_error ("\tdone\n");
    std::cout << "\t" <<  number_of_clusters << " clusters found" << std::endl
              << "\t<(execution time: " << euclidean_tt.toc() << " ms)>" << std::endl;
    clusters_cloud_vector.resize(number_of_clusters);

    std::cout << std::endl <<  "---> DOMINANT PLANE AND CLUSTERS EXTRACTION  total execution time: " << euclidean_tt.toc() << " ms" << std::endl << std::endl;

    if (number_of_clusters > 0)
    {
        // STEP KV5. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH *****************************************************************************************
        // ***********************************************************************************************************************************************************
        // The next step is implement for each found cluster. It deals with the determination of OUR-CVFH signature and woth the search of that signture in the DB.
        pcl::console::print_error ("\nSTEP KV5. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH ");
        pcl::console::print_error ("*******************************************************");
        pcl::console::print_error ("******************************************************\n");

                // Current cluster pointer
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster;
        // Current normals pointer
        pcl::PointCloud<pcl::Normal>::Ptr current_normals;
        // Current cluster with normals pointer
        pcl::PointCloud<pcl::PointNormal>::Ptr current_cluster_normals;
        // Current OUR-CVFH signature pointer
        pcl::PointCloud<pcl::VFHSignature308>::Ptr current_signature;
        // Vector of float containing the OUR-CVFH signature
        std::vector<float> current_signature_vector;
        current_signature_vector.resize(308);
        // Current transformation matrix
        Eigen::Matrix<float, 4, 4> current_pose;
        // Temporary <Object> instantiation
        Object current_object;
        // Path of the subfolder
        std::string search_path;  
        // Flag indicating correspondence found
        bool corr_found;

        // Time counter
        pcl::console::TicToc tt_search;
        tt_search.tic();

        for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices_vector.begin (); it != clusters_indices_vector.end (); ++it)
        {
            // OUR-CVFH estimation class instantiation
            OURCVFHEstimation ourcvfh_object;
            // DBManagement class instantiation
            DBManagement search_object;
            // Objects model DB base dir
            boost::filesystem::path DB_base_dir = "../ObjectDB/";

            // Resetting the cluster pointer;
            current_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
            // Resetting the current normals pointer
            current_normals.reset(new pcl::PointCloud<pcl::Normal>);
            // Resetting the current cluster with normals pointer
            current_cluster_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
            // Resetting the current signature pointer
            current_signature.reset(new pcl::PointCloud<pcl::VFHSignature308>);

            // By default, each time the loop is entered no correspondence is found
            corr_found = 0;
            
            // Loading of cluster informations and data from the original (segmented) cloud
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
               current_cluster->points.push_back (no_plane_cloud->points[*pit]); 
            current_cluster->width = current_cluster->points.size ();
            current_cluster->height = 1;
            current_cluster->is_dense = true;

            pcl::console::print_error("\n\tCLUSTER - %d (%d data points)", list_idx, current_cluster->points.size());
            pcl::console::print_error(" -------------------------------------------------------------------");
            pcl::console::print_error("-------------------------------------------------------------------\n");

            
            // STEP KV5.1 MLS FILTER APPLICATION AND NORMALS ESTIMATION ******************************************************************************************
            // ***************************************************************************************************************************************************
            // In this step a Voxel grid filter is applied in order to uniform the data points density
            tt.tic();
            pcl::console::print_error("\n\t\tSTEP KV5.1. MLS filter application and normals computation...");
                  
            processing_object.MLSFilterAndNormalsComputation(current_cluster,current_normals,current_cluster_normals);

            pcl::console::print_error ("\tdone\n");
            std::cout << "\t\t(" << current_cluster->points.size() << " data points)" << std::endl
                      << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

            
            // STEP KV5.2 OUR-CVFH SIGNATURE ESTIMATION ********************************************************************************************************
            // ***************************************************************************************************************************************************
            tt.tic();
            pcl::console::print_error ("\n\t\tSTEP KV5.2 OUR-CVFH signature estimation...");
          
            current_pose = ourcvfh_object.CloudOURCVFHComputation(current_cluster,current_normals,current_signature);


            pcl::console::print_error ("\tdone\n");
            std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;    

            // STEP KV5.2.1 OUR-CVFH SIGNATURE TRANSFORMATION TO FLOAT VECTOR ******************************************************************************************
            // *********************************************************************************************************************************************************
            // In this step the OUR-CVFH signature is tranformed in a float vector (attribute of the class Cloud)
            tt.tic();
            pcl::console::print_error ("\n\t\tSTEP KV5.2.1 OUR-CVFH signature transformation...");

            current_signature_vector = ourcvfh_object.OURCVFHSignatureTransformation(current_signature);

            pcl::console::print_error ("\tdone\n");      
            std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;    


            // STEP NS4.3. DB SEARCH ************************************************************************************************************************************
            // *********************************************************************************************************************************************************
            // In this step the main folder of the DB is searched in order to find a correspondence with the extracted cluster histogram. Each sub-folder is explored 
            // searching the correspondent Kd-Tree.            

            tt.tic();
            pcl::console::print_error ("\n\t\tSTEP KV5.3 DB SEARCH...");

            if (!boost::filesystem::exists (base_path_scene_DB) && !boost::filesystem::is_directory (base_path_scene_DB))
            {
                pcl::console::print_error("\n\t\t\t[FATAL ERROR] No DataBase found!\n");
                return;
            }

            // Check the number of elements of the reference chosen reference scene. It may happen that the reference scene is empty: in this case no DB search is 
            // performed and each cluster of the actual scene will be considered not part of the scene and marked as an outlier object.
            if (ref_subfolders.size() > 0)
            {
                for (int sub_idx = 0; sub_idx < ref_subfolders.size(); ++sub_idx)
                {
                    std::cout << "\n\t\tSearching the folder " << ref_subfolders[sub_idx].first << " ... " << std::flush;

                    if (!boost::filesystem::exists (ref_subfolders[sub_idx].first) && !boost::filesystem::is_directory (ref_subfolders[sub_idx].first))
                    {
                        pcl::console::print_error ("\n[ERROR] The ObjectDB folder you are considering does not exist!\n\n");
                        return;
                    }

                    // Updating search object attributes
                    search_object.UpdateObject(ref_subfolders[sub_idx].first);

                    // Searching the folder and updating the path vector
                    if (search_object.SearchTheDB(current_signature_vector))
                    {
                        // Update the vector containing objects that are part of the reference scene
                        scene_object.UpdateInnerObjectsVector(ref_subfolders[sub_idx].second, current_pose, current_cluster);
                        // Erase the searched subfolder from the list of possible objects to find
                        ref_subfolders.erase(ref_subfolders.begin()+sub_idx);

                        corr_found = 1;
                        break;
                    }

                    if (!corr_found)
                    {
                        std::cout << std::endl << "\t\tThe current cluster is not part of the reference scene!" 
                                  << std::endl << "\t\tIt has to be eliminated from the scene!" << std::endl;

                        // Update the vector containing objects that are not part of the reference scene
                        scene_object.UpdateOuterObjectsVector(ref_subfolders[sub_idx].second, current_pose, current_cluster);
                    }
                }
            }
            std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;
            ++list_idx;
        }
        std::cout << std::endl <<  "---> CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH total execution time: " << tt_search.toc() << " ms" << std::endl << std::endl;
    }
    else
      pcl::console::print_error("\t\t[WARNING] No clusters found! No new action will be performed!");



    // STEP KV6. SHOWING INFOS AND CLOUDS ***************************************************************************************************************************
    // **************************************************************************************************************************************************************
    
    pcl::console::print_error ("\nSTEP KV6. SHOWING RESULTS");
    pcl::console::print_error ("*************************************************************************");
    pcl::console::print_error ("***********************************************************************\n");

    // Printing infos about inner and outer objects
    pcl::console::print_error ("\tSTEP KV6.1. Printing inner and outer objects infos...\n");
    scene_object.ShowInfo();

    // Visualizing reference and actual scene
    pcl::console::print_error ("\n\tSTEP KV6.2. Visualizing reference and actual scenes...");
    scene_object.VisualizeActualScene();
    pcl::console::print_error ("\tdone\n\n");
}