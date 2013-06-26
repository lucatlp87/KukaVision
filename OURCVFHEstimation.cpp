// <OURCVFHEstimation> CLASS METHODS IMPLEMENTATION


#include <pcl/features/our_cvfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Cloud.h"
#include "OURCVFHEstimation.h"

// CONSTRUCTOR
OURCVFHEstimation::OURCVFHEstimation() {}

// DESTRUCTOR
OURCVFHEstimation::~OURCVFHEstimation() {}

// ##################################################################################################################################################################
// OUR-CVFH SIGNATURE ESTIMATION ####################################################################################################################################
// ##################################################################################################################################################################

// ESTIMATING THE SIGNATURE

void 
OURCVFHEstimation::CloudOURCVFHComputation(Cloud *input_cloud)
{	
		// Normals object pointer
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
	cloud_normals_ptr = input_cloud->GetNormals();
	// Point Cloud object pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	point_cloud_ptr = input_cloud->GetCloud();
	// Histogram object
	pcl::PointCloud<pcl::VFHSignature308> OURCVFH_histogram;

	// VFH estimator object 
	pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
	// Kd-Tree object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr ourcvfh_tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	// Initialization of VFH estimator object
	ourcvfh.setInputCloud (point_cloud_ptr);
	ourcvfh.setInputNormals (cloud_normals_ptr);
	ourcvfh.setSearchMethod (ourcvfh_tree);

	ourcvfh.setCurvatureThreshold (0.785); 		// 45° - maximum curvature between normals not to be considered edges
	ourcvfh.setClusterTolerance (0.3);			// 5 cm - maximum Euclidean distance between points to be considered part of the same cluster
	ourcvfh.setEPSAngleThreshold (0.5);		// 25° - maximum angular deviation between normals to be consdiered part of the same cluster
	ourcvfh.setMinPoints (50);					// minimum number of points of a cluster to be considered valid
	
	ourcvfh.setNormalizeBins (false);
	ourcvfh.setAxisRatio (0.9);				// maximum ratio between SGURF axis to avoid ambiguity

	// OUR-CVFH descriptors determination
	ourcvfh.compute (OURCVFH_histogram);

	OURCVFH_histogram.height = 1;
	OURCVFH_histogram.width = 1;

	input_cloud->SetOURCVFH(OURCVFH_histogram);
}