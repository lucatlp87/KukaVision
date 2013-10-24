// <OURCVFHEstimation> CLASS METHODS IMPLEMENTATION

#include <boost/shared_ptr.hpp>

#include <pcl/features/our_cvfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
OURCVFHEstimation::CloudOURCVFHComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals, 
										   pcl::PointCloud<pcl::VFHSignature308>::Ptr signature, 
										   std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > &transformation)
{	
	// OUR-CVFH estimator object 
	pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
	// Kd-Tree object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr ourcvfh_tree (new pcl::search::KdTree<pcl::PointXYZ>);

 	// Initialization of OUR-CVFH estimator object
	ourcvfh.setInputCloud (cluster);
	ourcvfh.setInputNormals (normals);
	ourcvfh.setSearchMethod (ourcvfh_tree);

	ourcvfh.setCurvatureThreshold (0.025f);
	ourcvfh.setClusterTolerance (0.015f);
	ourcvfh.setEPSAngleThreshold (0.13f);
	ourcvfh.setMinPoints (50);
	
	ourcvfh.setNormalizeBins (false);
	ourcvfh.setAxisRatio (0.98);

	// OUR-CVFH descriptors determination
	ourcvfh.compute (*signature);
	signature->width = signature->points.size();
	signature->height = 1;
	pcl::PointCloud<pcl::VFHSignature308>::Ptr test (new pcl::PointCloud<pcl::VFHSignature308>);

	// Transformation matrix extraction
	ourcvfh.getTransforms(transformation);
}

// TRANSFORMING THE SIGNATURE IN A <float> VECTOR
std::vector<float> 
OURCVFHEstimation::OURCVFHSignatureTransformation(pcl::PointCloud<pcl::VFHSignature308>::Ptr signature)
{
	// The function set the <point_cloud_OURCVFH_vector> attribute

	// Local point cloud
	pcl::PointCloud<pcl::VFHSignature308>::Ptr local_signature (new pcl::PointCloud<pcl::VFHSignature308>);
	*local_signature = *signature;
	// Point cloud fields object
	std::vector <pcl::PCLPointField> fields;
	// Output vector
	std::vector<float> output_vector;
	output_vector.resize(308);

	// Getting the histogram values from the cloud
	pcl::getFieldIndex (*local_signature, "vfh", fields);

	// Filling the vector
	for (int i = 0; i < 308; ++i)
		output_vector[i] = local_signature->points[0].histogram[i];

	return(output_vector);
}