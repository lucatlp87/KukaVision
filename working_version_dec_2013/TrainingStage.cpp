// <TrainingStage> CLASS METHODS DEFINITION

//#include <iostream>
//#include <string>
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

#include "TrainingStage.h"
#include "KinectAcquisition.h"
#include "CloudProcessing.h"
#include "OURCVFHEstimation.h"
#include "DBManagement.h"

// CONSTRUCTOR
TrainingStage::TrainingStage() {}

// DESTRUCTOR
TrainingStage::~TrainingStage() {}

// #################################################################################################################################################################
// TRAINING STAGE IMPLEMENTATION ###################################################################################################################################
// #################################################################################################################################################################

void
TrainingStage::RunStage()
{
	pcl::console::print_error("\n\n########################################################################");
	pcl::console::print_error("########################################################################\n");
	pcl::console::print_error("#################################################### TRAINING STAGE - OBJECT MODELS CREATION");
	pcl::console::print_error(" ###################################################\n");
    pcl::console::print_error("########################################################################");
	pcl::console::print_error("########################################################################\n");
    std::cout << "TRAINING STAGE aims to create a DataBase of object models." << std::endl
              << "Place the object on a rotating platform and di as many acquisition as the number of views you want to have for each object."
              << "The more are the acquisition the more the probability of matching increases." << std::endl << std::endl;

    // INITIALIZATION **********************************************************************************************************************************************
    // Acquired cloud container
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_acquisition (new pcl::PointCloud<pcl::PointXYZ>);
   	// Acquisition class instantiation
	KinectAcquisition acquisition_object;
	// Processing cloud class instantiation
	CloudProcessing processing_object;

    // Time counter
    pcl::console::TicToc tt;
    // *************************************************************************************************************************************************************                

<<<<<<< HEAD
    // STEP T1. ACQUISITION OF THE CLOUD FROM DEVICE ***************************************************************************************************************
	// *************************************************************************************************************************************************************
=======
     // STEP T1. ACQUISITION OF THE CLOUD FROM KINECT *******************************************************************************************************
	// *****************************************************************************************************************************************************
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
	// Be sure to have:
	//  - the Kinect sensor plugged in
	//  - the object of which you want to create the model in the center of the scene with no other object
    // In order to acquire the cloud, the user has to stop the visualizer (pressing 'q')
        
	tt.tic();
	pcl::console::print_error ("\nSTEP T1. POINT CLOUD ACQUISITION FROM KINECT ");
	pcl::console::print_error ("**************************************************");
	pcl::console::print_error ("*************************************************\n");
	       
	// Acquiring the cloud from Kinect sensor
	// acquisition_object.AcquireCloudKinect(kinect_acuqisition);
	pcl::io::loadPCDFile("real_scanner/containner_pasta/container_pasta_top_135.pcd",*kinect_acquisition);

    std::cout << std::endl <<  "---> POINT CLOUD ACQUISITION FROM KINECT total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

    // STEP T2. PASS-THROUGH FILTER APPLICATION ************************************************************************************************************
    // *****************************************************************************************************************************************************
<<<<<<< HEAD
    // The pass-through filter allows to isolate only the worktable. This is fundamental because the next step (cluster extraction) requires that the scene 
    // contains only one dominant horizontal plane
=======
    
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
    tt.tic();
    pcl::console::print_error ("\nSTEP T2. PASS-THROUGH FILTER APPLICATION ");
    pcl::console::print_error ("****************************************************");
    pcl::console::print_error ("***************************************************\n");

    // Filter application (the filter parameters are fitted on the used tabletop) 
    processing_object.PassThroughFilter(kinect_acquisition);

    std::cout << std::endl << "---> PASS-THROUGH FILTER APPLICATION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
            
	// STEP T3. DOMINANT PLANE AND CLUSTERS EXTRACTION *****************************************************************************************************
    // *****************************************************************************************************************************************************
    tt.tic();
    pcl::console::print_error ("\nSTEP T3. DOMINANT PLANE AND CLUSTERS EXTRACTION ");
    pcl::console::print_error ("************************************************");
    pcl::console::print_error ("************************************************\n");

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
    // Visualizer legend
    std::string win_legend_init = "TRAINING STAGE - CLUSTERS STORAGE\nCluster ";
    std::string win_legend_end = "/";

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
    SAC_filter.setDistanceThreshold (0.006);

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
    cluster_exctraction.setClusterTolerance (0.01);
    cluster_exctraction.setMinClusterSize (500);
    cluster_exctraction.setSearchMethod (segmentation_tree);
    cluster_exctraction.setInputCloud (no_plane_cloud);

    // Cluster extraction
    // Each element of the vector <clusters_indices_vector> corresponds to a cluster of the cloud
    cluster_exctraction.extract (clusters_indices_vector);

    number_of_clusters = clusters_indices_vector.size();
    win_legend_end.append(boost::lexical_cast<std::string>(number_of_clusters));

    pcl::console::print_error ("\tdone\n");
    std::cout << "\t" <<  number_of_clusters << " clusters found" << std::endl
              << "\t<(execution time: " << euclidean_tt.toc() << " ms)>" << std::endl;     
	
    std::cout << std::endl << "---> DOMINANT PLANE AND CLUSTER EXTRACTION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
    
    if (number_of_clusters > 0)
    {
       	// STEP T4. CLUSTER VISUALIZATION AND STORAGE ********************************************************************************************************
        // *****************************************************************************************************************************************************
        // The next step is implement for each found cluster. It deals with visualization of the cluster (wrt the orignal acquired cloud) and the user choice 
        // about saving it or not.
        pcl::console::print_error ("\n\tSTEP T4. CLUSTERS VISUALIZATION AND STORAGE"); 
        pcl::console::print_error(" **********************************************");
		pcl::console::print_error("**********************************************\n");
        	
       	// Current cluster pointer
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster;
        // Current normals pointer
        pcl::PointCloud<pcl::Normal>::Ptr current_normals;
        // Current cluster with normals pointer
        pcl::PointCloud<pcl::PointNormal>::Ptr current_cluster_normals;
        // Current OUR-CVFH signature pointer
        pcl::PointCloud<pcl::VFHSignature308>::Ptr current_signature;
        // Current pose matrix
        std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > current_transformation;
        // First storage flag
        bool first_storage = 1;

       	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices_vector.begin (); it != clusters_indices_vector.end (); ++it)
        {
            // Text legend update
            std::string win_legend = win_legend_init;
            win_legend.append(boost::lexical_cast<std::string>(list_idx)).append(win_legend_end);
            // Resetting the cluster pointer;
			current_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
			// Resetting the current normals pointer
			current_normals.reset(new pcl::PointCloud<pcl::Normal>);
			// Resetting the current cluster with normals pointer
			current_cluster_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
			// Resetting the current signature pointer
			current_signature.reset(new pcl::PointCloud<pcl::VFHSignature308>);
    
            // Loading of cluster informations and data from the original (segmented) cloud
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	            current_cluster->points.push_back (no_plane_cloud->points[*pit]); 
	        current_cluster->width = current_cluster->points.size ();
	        current_cluster->height = 1;
	        current_cluster->is_dense = true;

	        // Visualization object
	        boost::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer (new pcl::visualization::PCLVisualizer ("Cluster Viewer"));         
	        cluster_viewer->initCameraParameters ();
	        cluster_viewer->addCoordinateSystem (1.0);
	        cluster_viewer->setBackgroundColor (0, 0, 0);
	        cluster_viewer->setWindowBorders(true);
	        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue (kinect_acquisition, 0, 0, 255);

            // Updated folder
            std::string updated_dir;
            // User choice
            std::string save_cloud;

            pcl::console::print_error("\n\tCLUSTER - %d (%d data points)", list_idx, current_cluster->points.size());
            pcl::console::print_error(" ----------------------------------------------------");
			pcl::console::print_error("----------------------------------------------------\n");
	            
            // VISUALIZING THE CLOUD
            std::cout << " \tCluster visualization " << std::flush;
            pcl::console::print_error ("\t(Please close the visualizer to continue)\n");
		              
            cluster_viewer->addPointCloud<pcl::PointXYZ> (kinect_acquisition, "Original");
            cluster_viewer->addPointCloud<pcl::PointXYZ> (current_cluster, color_blue, "Cloud");

            cluster_viewer->resetCamera();
            cluster_viewer->resetCameraViewpoint("Cloud");
            cluster_viewer->resetCameraViewpoint("Original");
            
            cluster_viewer->addText(win_legend, 10, 10, 1, 0, 0, "AcquiredCloudText");
            cluster_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText");
            cluster_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");

            while (!cluster_viewer->wasStopped ())
            { 
                cluster_viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (1000));
            }
            cluster_viewer->close();

            // User choice parsing
            while(true)
            {
            	std::cout << "\tDo you want to save this cloud? ";
            	std::cin >> save_cloud;
	            	if (!save_cloud.compare("y") || !save_cloud.compare("n"))
		           		break;
		           	else
		           		pcl::console::print_error ("\n\t\t[COMMAND ERROR] Please insert <y> or <n>\n\n");
		    }
		            
		    // If the user decides not to save the cloud, the first element of the return vector is settet to the value "NOT_SAVED" and then the loop considers the
            // next cluster.
            if (save_cloud.compare("y"))
                updated_dir = "NOT_SAVED";
            else
            {
            	// STEP T4.1. MLS FILTER APPLICATION AND NORMALS ESTIMATION ******************************************************************************************
	            // ***************************************************************************************************************************************************
	            // In this step a Voxel grid filter is applied in order to uniform the data points density
	            tt.tic();
	            pcl::console::print_error("\n\t\tSTEP T4.1. MLS filter application and normals computation...");
                
                // Determining the dilation iteration number
                int dilation_it;
                dilation_it = ceil(current_cluster->points.size()*15/232000);
                if(dilation_it < 10)
                    dilation_it = 10;
			            
		        processing_object.MLSFilterAndNormalsComputation(current_cluster,current_normals,current_cluster_normals,dilation_it);

	            pcl::console::print_error ("\tdone\n");
	            std::cout << "\t\t(" << current_cluster->points.size() << " data points)" << std::endl
	                      << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

	            // boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer_2 (new pcl::visualization::PCLVisualizer ("Scene Viewer"));     
             //                scene_viewer_2->initCameraParameters ();
             //                scene_viewer_2->addCoordinateSystem (1.0);
             //                scene_viewer_2->setBackgroundColor (0, 0, 0);
             //                scene_viewer_2->setWindowBorders(true);
             //                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_midori_2 (current_cluster, 227, 249, 136);

             //                scene_viewer_2->addPointCloud<pcl::PointXYZ> (current_cluster, color_midori_2, "Scene");
             //                scene_viewer_2->resetCameraViewpoint("Scene");
             //                scene_viewer_2->addText("CHOSEN REFERENCE SCENE", 10, 10, 1, 0, 0, "SceneText");
             //                scene_viewer_2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Scene");
             //                scene_viewer_2->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "SceneText");

             //                while (!scene_viewer_2->wasStopped ())
             //                { 
             //                    scene_viewer_2->spinOnce (100);
             //                    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
             //                }
             //                scene_viewer_2->close();

                // STEP T4.2. OUR-CVFH SIGNATURE ESTIMATION ********************************************************************************************************
       			// ***************************************************************************************************************************************************
	            tt.tic();
	            pcl::console::print_error ("\n\t\tSTEP T4.2. OUR-CVFH signature estimation...");
	            // OUR-CVFH estimation class 
	            OURCVFHEstimation ourcvfh_object;

	            ourcvfh_object.CloudOURCVFHComputation(current_cluster,current_normals,current_signature, current_transformation);

	            pcl::console::print_error ("\tdone\n");
                std::cout << "\t\t" << current_signature->points.size() << " smooth region(s) found" << std::endl;
	            std::cout << "\t\t<(execution time: " << tt.toc() << " ms)>" << std::endl;


                // STEP T4.3. SAVING THE CLOUD ***********************************************************************************************************************
                // ****************************************************************************************************************************************************
                pcl::console::print_error ("\n\t\tSTEP T4.3. Cloud storage...\n");
                std::cout << std::endl << current_transformation[0] << std::endl << std::endl;
                updated_dir = processing_object.SaveCloud(current_cluster_normals,current_signature, current_transformation);

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

            // Update the index of the current cluster
            ++list_idx;
        }
        std::cout << std::endl << "---> DOMINANT PLANE AND CLUSTERS EXTRACTION total execution time: " << tt.toc() << " ms" << std::endl << std::endl;

        // STEP T5. KDTREE UPDATE *********************************************************************************************************************************
        // ********************************************************************************************************************************************************

        tt.tic();
        pcl::console::print_error ("\nSTEP T5. KD-TREES UPDATE");
        pcl::console::print_error(" ***********************************************************");
        pcl::console::print_error("************************************************************\n");
          
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
                pcl::console::print_error ("\tSTEP T5.1. OUR-CVFH histrogram conversion to Flann format...");

                // Flann conversion
                update_object.OURCVFHFlannConversion(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                // STEP T4.2 OUR-CVFH HISTOGRAM FILE NAME STORAGE ******************************************************************************************************
                // *****************************************************************************************************************************************************
                // The second step consist in adding (in append mode) the new histrogrms paths to the file containing names of hall istograms of the current subfolder.

                tt_update.tic();
                pcl::console::print_error ("\n\tSTEP T5.2. OUR-CVFH histograms path add...");

                // Name list update
                update_object.HistogramNameStorage(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                // STEP T4.3 CREATION, BUILD AND STORAGE OF THE KD-TREE ***********************************************************************************************
                // ****************************************************************************************************************************************************
                // The last step consists in creating and build a tree index about all elements of the current subfolder.

                tt_update.tic();
                pcl::console::print_error ("\n\tSTEP T5.3. Kd-Tree index build and storage...");

                // Index building and storage
                update_object.BuildOURCVFHTreeIndex(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;
            }
        }
        std::cout << std::endl << "---> KD-TREES UPDATE total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
    }
    else
   		pcl::console::print_error("\t\t[WARNING] No clusters found!");
}