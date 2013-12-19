// <NewSceneStage> CLASS METHODS DEFINITION

#include <iostream>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/StdVector>

#include <flann/flann.hpp>

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

#include "NewSceneStage.h"
#include "KinectAcquisition.h"
#include "SceneManagement.h"
#include "CloudProcessing.h"
#include "OURCVFHEstimation.h"
#include "DBManagement.h"

// DEFINITION OF AUXILIARY STRUCTURES
// Best match structure
typedef std::pair<std::string, float> match_pair;

// CONSTRUCTOR
NewSceneStage::NewSceneStage() {}

// DESTRUCTOR
NewSceneStage::~NewSceneStage() {}

// #################################################################################################################################################################
// NEWSCENE STAGE IMPLEMENTATION ###################################################################################################################################
// #################################################################################################################################################################

void
NewSceneStage::RunStage()
{
    pcl::console::print_error("\n\n#######################################################################################");
	  pcl::console::print_error("#######################################################################################\n");
	  pcl::console::print_error("################################################################## NEW SCENE STAGE - OBJECT MODELS CREATION");
	  pcl::console::print_error(" ##################################################################\n");
    pcl::console::print_error("#######################################################################################");
	  pcl::console::print_error("#######################################################################################\n");
	  std::cout << "Please set the scene and start this procedure in order to register a new reference scene" << std::endl << std::endl;

    // INITIALIZATION
    // Acquired cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
  	// Acquisition class instantiation
  	KinectAcquisition acquisition_object;
  	// <SceneManagement> object instantiation
  	SceneManagement scene_object;
  	// Processing class instantiation
  	CloudProcessing processing_object;
  	// Current object id
  	int current_obj_id = 1;

  	// Time counter
  	pcl::console::TicToc tt;

  	// STEP NS1. ACQUISITION OF THE CLOUD FROM KINECT ***********************************************************************************************************
  	// *********************************************************************************************************************************************************
	  // Be sure to have:
  	//  - the Kinect sensor plugged in
  	//  - the new reference scene you wanto to acquire in the visible range of the Kinect
        
  	tt.tic();
  	pcl::console::print_error ("\nSTEP NS1. POINT CLOUD ACQUISITION FROM KINECT ");
  	pcl::console::print_error("****************************************************************");
    pcl::console::print_error("****************************************************************\n");

  	// acquisition_object.AcquireCloudKinect(kinect_acquisition);
	  pcl::io::loadPCDFile("scene.pcd",*kinect_acquisition);

  	std::cout << std::endl <<  "---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

  	
    // STEP NS2. PASS-THROUGH FILTER APPLICATION **********************************************************************************************************
  	// **********************************************************************************************************************************************************

  	tt.tic();
  	pcl::console::print_error ("\nSTEP NS2. PASS-THROUGH FILTER APPLICATION ");
  	pcl::console::print_error ("******************************************************************");
    pcl::console::print_error ("******************************************************************\n");

  	// Filter application
  	// processing_object.PassThroughFilter(kinect_acquisition);
    // Saving the cloud of the scene in the <SceneManagement> object
    scene_object.SetRefScene(kinect_acquisition);

  	std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

  	
    // STEP NS3. DOMINANT PLANE AND CLUSTERS EXTRACTION *****************************************************************************************************
    // *****************************************************************************************************************************************************
    tt.tic();
    pcl::console::print_error ("\nSTEP NS3. DOMINANT PLANE AND CLUSTERS EXTRACTION ");
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
    cluster_exctraction.setMinClusterSize (50);
    cluster_exctraction.setSearchMethod (segmentation_tree);
    cluster_exctraction.setInputCloud (no_plane_cloud);

    // Cluster extraction
    // Each element of the vector <clusters_indices_vector> corresponds to a cluster of the cloud
    cluster_exctraction.extract (clusters_indices_vector);

    number_of_clusters = clusters_indices_vector.size();

    pcl::console::print_error ("\tdone\n");
    std::cout << "\t" <<  number_of_clusters << " clusters found" << std::endl
              << "\t<(execution time: " << euclidean_tt.toc() << " ms)>" << std::endl;

    std::cout << std::endl << "---> DOMINANT PLANE AND CLUSTER EXTRACTION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

    if (number_of_clusters > 0)
    {
        // STEP NS4. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH *****************************************************************************************
    	  // ***********************************************************************************************************************************************************
    	  // The next step is implement for each found cluster. It deals with the determination of OUR-CVFH signature and woth the search of that signture in the DB.
    	  pcl::console::print_error ("\nSTEP NS4. CLUSTERS OUR-CVFH SIGNATURE DETERMINATION AND DB SEARCH ");
    	  pcl::console::print_error ("******************************************************");
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
        std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > current_pose;
        // Temporary <Object> instantiation
      	Object current_object;
        // Current match pair object
        match_pair current_match;
        // Best match percentage
        float best_match_percentage;
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
            // Vector containing matches
            std::vector<match_pair> match_vector;

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

			      
            // STEP NS4.1. MLS FILTER APPLICATION AND NORMALS ESTIMATION ******************************************************************************************
	          // ***************************************************************************************************************************************************
	          // In this step a Voxel grid filter is applied in order to uniform the data points density
	          tt.tic();
	          pcl::console::print_error("\n\t\tSTEP NS4.1. MLS filter application and normals computation...");
			            
		        processing_object.MLSFilterAndNormalsComputation(current_cluster,current_normals,current_cluster_normals);

	          pcl::console::print_error ("\tdone\n");
	          std::cout << "\t\t(" << current_cluster->points.size() << " data points)" << std::endl
	                    << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

	          
            // STEP T4.2. OUR-CVFH SIGNATURE ESTIMATION ********************************************************************************************************
       		  // ***************************************************************************************************************************************************
	          tt.tic();
	          pcl::console::print_error ("\n\t\tSTEP NS4.2. OUR-CVFH signature estimation...");
	        
	          ourcvfh_object.CloudOURCVFHComputation(current_cluster,current_normals,current_signature, current_pose);

  	        pcl::console::print_error ("\tdone\n");
            std::cout << "\t\t" << current_signature->points.size() << " smooth region(s) found" << std::endl;
	          std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;	  

            // The DB search has to be performed once for every found smooth region
            pcl::PointCloud<pcl::VFHSignature308>::Ptr smooth_signature (new pcl::PointCloud<pcl::VFHSignature308>);
            smooth_signature->height = 1; smooth_signature->width = 1;
            smooth_signature->points.resize(1);
            for (size_t smooth_idx = 0; smooth_idx < current_signature->points.size(); ++ smooth_idx)
            {
                best_match_percentage = 0;

                if (!corr_found)
                {
                    pcl::console::print_error ("\n\t\tPROCESSING SIGNATURE %d OF %d ",smooth_idx+1,current_signature->points.size());
                    pcl::console::print_error ("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                    smooth_signature->points[0] = current_signature->points[0,smooth_idx];

                    // STEP NS4.2.1 OUR-CVFH SIGNATURE TRANSFORMATION TO FLOAT VECTOR ******************************************************************************************
                    // *********************************************************************************************************************************************************
                    // In this step the OUR-CVFH signature is tranformed in a float vector (attribute of the class Cloud)
                    tt.tic();
                    pcl::console::print_error ("\t\t\tSTEP NS4.2.1 OUR-CVFH signature transformation...");

                    current_signature_vector = ourcvfh_object.OURCVFHSignatureTransformation(smooth_signature);

                    pcl::console::print_error ("\tdone\n");      
                    std::cout << "\t\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;    
                    
                    // STEP NS4.3. DB SEARCH ************************************************************************************************************************************
                    // *********************************************************************************************************************************************************
                    // In this step the main folder of the DB is searched in order to find a correspondence with the extracted cluster histogram. Each sub-folder is explored 
                    // searching the correspondent Kd-Tree.            

                    tt.tic();
                    pcl::console::print_error ("\n\t\t\tSTEP NS4.3. DB SEARCH...");

                    if (!boost::filesystem::exists (DB_base_dir) && !boost::filesystem::is_directory (DB_base_dir))
                    {
                        pcl::console::print_error("\n\t\t\t\t[FATAL ERROR] No DataBase found!\n");
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
                            std::cout << "\n\t\t\tSearching the folder " << search_path << " ... " << std::flush;

                            // Updating search object attributes
                            search_object.UpdateObject(search_path);

                            // Searching the folder and updating the path vector
                            current_match = search_object.SearchTheDB(current_signature_vector, current_cluster->points.size());
                            if (current_match.second > 0)
                            {
                                if (current_match.second > best_match_percentage)
                                {   
                                    //Retrieving the real object pose
                                    std::ifstream matrix_file_ptr;
                                    float matrix_element;
                                    Eigen::Matrix4f T_cf;
                                    Eigen::Matrix4f current_obj_pose;

                                    matrix_file_ptr.open (current_match.first.c_str());    
                                    for (int matrix_row = 0; matrix_row < 4; ++matrix_row)
                                        for (int matrix_col = 0; matrix_col < 4; ++matrix_col)
                                        {
                                            matrix_file_ptr >> matrix_element;
                                            T_cf(matrix_row,matrix_col) = matrix_element;
                                        }
                                    matrix_file_ptr.close();
                                    current_obj_pose = current_pose[smooth_idx]*T_cf;

                                    // Update the object list
                                    current_object.SetObjectID(current_obj_id);
                                    ++current_obj_id;
                                    current_object.SetObjectType(ss.str().substr(ss.str().find_last_of("/")+1));
                                    current_object.SetObjectModelPath(search_path);
                                    current_object.SetObjectRefPose(current_obj_pose);
                                    current_object.SetObjectCloud(current_cluster);
                                    // Update the best match percentage
                                    best_match_percentage = current_match.second;
                                
                                    pcl::console::print_error("\t\t\t\t\tNew best match!\n");
                                    corr_found = 1;
                                    ++list_idx;
                                }
                            }
                        }
                    }
                }
            }
            // Saving the object to the object list of the new scene
            scene_object.InsertObject(current_object);

            // If there is a cloud corresponding to an object not identified in the Object DB the user will be asked about it:
            // he can choose to save it as a new object or ignore it (in this case the cloud becomes "invisible" in the reference scene)
            if (!corr_found)
            {
                // Visualization object instantiation
                boost::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer (new pcl::visualization::PCLVisualizer);
                // Usere choice
                std::string save_new_model;

                // Visualization object initialization
                cluster_viewer->initCameraParameters ();
                cluster_viewer->addCoordinateSystem (1.0);
                cluster_viewer->setBackgroundColor (0, 0, 0);
                cluster_viewer->setWindowBorders(true);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue (current_cluster, 0, 0, 255);
              
                pcl::console::print_error("\n\t[WARNING] The considered cluster has no correspondences in the DB!\n");
                pcl::console::print_error("\tPlease close the visualizer to continue...\n");

                cluster_viewer->addPointCloud<pcl::PointXYZ> (scene_object.GetRefScene(),"Original");
                cluster_viewer->addPointCloud<pcl::PointXYZ> (current_cluster, color_blue, "Cloud");
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

                while(true)
                {
                    std::cout << "\tDo you want to save it as a new model? " << std::flush;
                    std::cin >> save_new_model;
                    if (!save_new_model.compare("y") || !save_new_model.compare("n"))
                        break;
                    else
                        pcl::console::print_error ("\n\t\t[COMMAND ERROR] Please insert <y> or <n>\n\n");
                }

                if(!save_new_model.compare("y"))
                {
                    // Update object
                    DBManagement new_model_update;
                    // Time counter
                    pcl::console::TicToc tt_update;

                    // SAVING THE CLOUD **************************************************************************************************************************************
                    pcl::console::print_error ("\n\tSaving the cloud...\n");
                    search_path = processing_object.SaveCloud(current_cluster_normals,current_signature);

                    // UPDATING THE FOLDER ***********************************************************************************************************************************
                    std::cout << std::endl << "\tUPDATING THE FOLDER " << search_path << std::flush;

                    // Update all paths and search the actual subfolder for .pcd files.
                    new_model_update.UpdateObject(search_path);
                    std::cout << "\t(" << search_object.GetFoundSignatureNumber() << " histograms found)" << std::endl;
                        
                    // STEP NS4.3.1. OUR-CVFH HISTOGRAM CONVERSION TO FLANN FORMAT *********************************************************************************************
                    // *****************************************************************************************************************************************************
                    // The first step of the update procedure deals with the conversion of OUR-CVFH histogram point cloud into a Flann matrix. Once the matrix is created 
                    // and filled it is saved in the current subfolder of the DB.
                    tt_update.tic();
                    pcl::console::print_error ("\t\tSTEP NS4.3.1. OUR-CVFH histrogram conversion to Flann format...");

                    // Flann conversion
                    new_model_update.OURCVFHFlannConversion(search_path);

                    pcl::console::print_error ("\tdone\n");
                    std::cout << "\t\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                    // STEP NS3.3.2. OUR-CVFH HISTOGRAM FILE NAME STORAGE ******************************************************************************************************
                    // *****************************************************************************************************************************************************
                    // The second step consist in adding (in append mode) the new histrogrms paths to the file containing names of hall istograms of the current subfolder.

                    tt_update.tic();
                    pcl::console::print_error ("\n\t\tSTEP NS4.3.2. OUR-CVFH histograms path add...");

                    // Name list update
                    new_model_update.HistogramNameStorage(search_path);

                    pcl::console::print_error ("\tdone\n");
                    std::cout << "\t\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                    // STEP NS4.3.3. CREATION, BUILD AND STORAGE OF THE KD-TREE ***********************************************************************************************
                    // ****************************************************************************************************************************************************
                    // The last step consists in creating and build a tree index about all elements of the current subfolder.

                    tt_update.tic();
                    pcl::console::print_error ("\n\t\tSTEP NS4.3.3. Kd-Tree index build and storage...");

                    // Index building and storage
                    new_model_update.BuildOURCVFHTreeIndex(search_path);

                    pcl::console::print_error ("\tdone\n");
                    std::cout << "\t\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;
                 
                    // Update the object list
                    current_object.SetObjectID(current_obj_id);
                    ++current_obj_id;
                    current_object.SetObjectModelPath(search_path);
                    search_path.erase(search_path.end()-1);
                    current_object.SetObjectType(search_path.substr(search_path.find_last_of("/")+1));
                    // If the object is a new one, the pose referred to the first smooth region is taken as reference pose
                    current_object.SetObjectRefPose(current_pose[0]);
                    current_object.SetObjectCloud(current_cluster);

                    scene_object.InsertObject(current_object);

                    ++list_idx;
                } 
                else
                {
                    pcl::console::print_error("\n\t[WARNING] The considered cluster will not be part of the reference scene!\n");
                }
            }
	      }
        std::cout << std::endl <<  "---> DB SEARCH total execution time: " << tt_search.toc() << " ms" << std::endl << std::endl;
    }
    else
 		  pcl::console::print_error("\t\t[WARNING] No clusters found! No new reference scene will be added to the DB!");

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

    scene_object.VisualizeRefScene();
}
