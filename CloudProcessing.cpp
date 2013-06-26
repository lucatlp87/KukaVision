// <CloudProcessing> CLASS METHODS IMPLEMENTATION

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

#include "Cloud.h"
#include "CloudProcessing.h"

// CONSTRUCTOR
CloudProcessing::CloudProcessing() {}

// DESTRUCTOR
CloudProcessing::~CloudProcessing() {}

// ##################################################################################################################################################################
// POINT CLOUD FILTERS ##############################################################################################################################################
// ##################################################################################################################################################################

// PASS-THROUGH FILTER **********************************************************************************************************************************************
void 
CloudProcessing::PassThroughFilter(Cloud input_cloud)
{
	// The function deals with the application of a Pass-through filter in order to extract from the acquired point cloud the region corresponding to the tabletop

	// Cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ptr = input_cloud.GetCloud();
	// Pass-through filter object
	pcl::PassThrough<pcl::PointXYZ> PTfilter;

	// Filter object initialization
	PTfilter.setInputCloud (cloud_ptr);
	PTfilter.setFilterFieldName ("x");
  	PTfilter.setFilterLimits (0.0, 1.0);
  	PTfilter.setFilterFieldName ("y");
  	PTfilter.setFilterLimits (0.0, 1.0);
  	PTfilter.setFilterFieldName ("z");
  	PTfilter.setFilterLimits (0.0, 1.0);

  	// Filter application
  	PTfilter.filter (*cloud_ptr);
  	input_cloud.SetCloud(*cloud_ptr);
}

// VOXEL GRID FILTER ************************************************************************************************************************************************
void 
CloudProcessing::MLSFilterAndNormalsComputation(Cloud *input_cloud)
{
	// The function deals with the application of a Voxel Grid Filter with leaf size of 3 mm (the best resolution that Kinct sensor offers)

	// Point cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ptr = input_cloud->GetCloud();
	// Normals cloud pointer
	pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
	// PointNormal cloud
	pcl::PointCloud<pcl::PointNormal> filtered_cloud;

  	// Instantiation of the MLS object 
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> MLSFilter;
	 // KD-Tree
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Filter object initialization
    MLSFilter.setInputCloud (cloud_ptr);
    MLSFilter.setComputeNormals(true);
    MLSFilter.setPolynomialFit(true);
    MLSFilter.setSearchMethod(tree);
    MLSFilter.setSearchRadius(0.03);
	MLSFilter.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
 	MLSFilter.setDilationIterations (2);
	MLSFilter.setDilationVoxelSize (0.005f);

	// Filter application
  	MLSFilter.process(filtered_cloud);

  	// Storage of results in the <Cloud> class in input 
  	cloud_ptr->points.resize(filtered_cloud.points.size());
  	normals_ptr->points.resize(filtered_cloud.points.size());

	for (size_t i = 0; i < filtered_cloud.points.size(); i++) 
	{
		// Copying PointXYZ data
    	filtered_cloud.points[i].x = cloud_ptr->points[i].x;
    	filtered_cloud.points[i].y = cloud_ptr->points[i].y;
    	filtered_cloud.points[i].z = cloud_ptr->points[i].z;
    	// Copying of Normal data
    	filtered_cloud.points[i].normal_x = normals_ptr->points[i].normal_x;
    	filtered_cloud.points[i].normal_y = normals_ptr->points[i].normal_y;
    	filtered_cloud.points[i].normal_z = normals_ptr->points[i].normal_z;
    }

    input_cloud->SetCloud(*cloud_ptr);
    input_cloud->SetNormals(*normals_ptr);
    input_cloud->SetCloudWithNormals(filtered_cloud);
}

// ##################################################################################################################################################################
// POINT CLOUD PROCESSING ###########################################################################################################################################
// ##################################################################################################################################################################

// CLUSTER EXTRACTION ***********************************************************************************************************************************************
std::vector<Cloud, Eigen::aligned_allocator<Cloud> > 
CloudProcessing::ExtractClustersFromCloud(Cloud input_cloud)
{
	// Point cloud after the planar components segmentation and extraction
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	segmented_cloud = input_cloud.GetCloud();

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
	// Vector containing extracted clusters
	std::vector<Cloud, Eigen::aligned_allocator<Cloud> > found_clusters;

   	// Time counter
   	pcl::console::TicToc tt;

   	// Number of found clusters 
   	int number_of_clusters;

	// MAIN LOOP ***************************************************************************************************************************************************
	// The main loop includes:
	// 		- removing the planar cluster corresponding to the tabletop of the scene
	// 		- segmenting and clustering the cloud 
	// 		- for each found cluster
	// 			- visualize it
	// 			- choose if save it or not. If yes
	// 				- compute normals
	// 				- compute VFH signature
	// 				- save cloud with normals anf VFH signature cloud
	

	// PLANAR CLUSTER ELIMINATION **********************************************************************************************************************************
	// *************************************************************************************************************************************************************
	// The first step consists in extracting the dominant plane cluster from the input cloud by using RANSAC algorithm and a predefined plane model. 

	tt.tic();
	pcl::console::print_error ("\tDominant plane cluster extraction...");

	// Segmentation object initialization
	SAC_filter.setModelType(pcl::SACMODEL_PLANE);
	SAC_filter.setOptimizeCoefficients (true);
	SAC_filter.setMethodType (pcl::SAC_RANSAC);
	SAC_filter.setMaxIterations (100);
	SAC_filter.setDistanceThreshold (0.02);

    // Segment the dominant plane cluster
  	SAC_filter.setInputCloud (segmented_cloud);
  	SAC_filter.segment (*planar_inliers, *planar_coefficients);
	  	
  	if (planar_inliers->indices.size () == 0)
  	{
  		pcl::console::print_error ("\n\t\t[SEGMENTATION ERROR] Could not estimate a plane model for the given dataset");	
  		pcl::console::print_error ("\n\t\tPlease consider a new dataset or change the segmentation model and try again\n");
		Cloud seg_error;
		seg_error.SetSegError(1);
		found_clusters.push_back(seg_error);
		return(found_clusters);
  	}

	// Remove the planar cluster from the input cloud
    planar_inliers_extraction.setInputCloud (segmented_cloud);
    planar_inliers_extraction.setIndices (planar_inliers);
    planar_inliers_extraction.setNegative (true);
    planar_inliers_extraction.filter (*segmented_cloud);

    pcl::console::print_error ("	done\n");
    std::cout << "\tThe remaining cloud has " << segmented_cloud->points.size() << " points" << std::endl
			  << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

	// EUCLIDEAN CLUSTER EXTRACTION ********************************************************************************************************************************
	// *************************************************************************************************************************************************************
	// The second step consists in  extracting all clusters of the remaining cloud. Clusters have to respect certain properties to be valid.

	tt.tic();
	pcl::console::print_error ("\n\tEuclidean cluster extraction...");
				
	// K-d tree initialization
	segmentation_tree->setInputCloud (segmented_cloud);
	// Cluster extraction object initializations
	cluster_exctraction.setClusterTolerance (0.02); // 2cm
	cluster_exctraction.setMinClusterSize (1000);
	cluster_exctraction.setSearchMethod (segmentation_tree);
	cluster_exctraction.setInputCloud (segmented_cloud);

	// Cluster extraction
	// Each element of the vector <clusters_indices_vector> corresponds to a cluster of the cloud
	cluster_exctraction.extract (clusters_indices_vector);
	number_of_clusters = clusters_indices_vector.size();

	pcl::console::print_error ("\tdone\n");
	std::cout << "\t" <<  number_of_clusters << " clusters found" << std::endl
			  << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

	if (number_of_clusters > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices_vector.begin (); it != clusters_indices_vector.end (); ++it)
		{
			// Each time that the <for> loop is entered a new point cloud object is created so that the previous cluster is no more considered
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			Cloud new_cloud;

			// Loading of cluster informations and data from the original (segmented) cloud
	    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    		cloud_cluster->points.push_back (segmented_cloud->points[*pit]); 
	    	cloud_cluster->width = cloud_cluster->points.size ();
	    	cloud_cluster->height = 1;
	    	cloud_cluster->is_dense = true;
	    	
	    	// Updating the output vector
	    	new_cloud.SetCloud(*cloud_cluster);
	    	found_clusters.push_back(new_cloud);
		}
		return(found_clusters);
	}
	else
	{
		pcl::console::print_error("\t\t[WARNING] No clusters found!");
		Cloud no_cluster;
		no_cluster.SetNoClusters(1);
  		found_clusters.push_back(no_cluster);
  		return(found_clusters);
	}
}


