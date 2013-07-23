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
Eigen::Matrix<float, 4, 4>
OURCVFHEstimation::CloudOURCVFHComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals, 
										   pcl::PointCloud<pcl::VFHSignature308>::Ptr signature)
{	
	// Vector containing all the tranformation matrices
	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > transformation;
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
	ourcvfh.setMinPoints (30);
	
	ourcvfh.setNormalizeBins (false);
	ourcvfh.setAxisRatio (0.98);

	// OUR-CVFH descriptors determination
	ourcvfh.compute (*signature);

	// Transformation matrix extraction
	ourcvfh.getTransforms(transformation);

	if (signature->height != 1)
		signature->height = 1;
	if (signature->width != 1)
		signature->width = 1;
	if (signature->points.size() != 1)
		signature->points.resize(1);

	return (transformation[0]);
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
	std::vector <sensor_msgs::PointField> fields;
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