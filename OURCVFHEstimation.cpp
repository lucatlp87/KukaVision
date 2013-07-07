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
										   pcl::PointCloud<pcl::VFHSignature308>::Ptr signature)
{	
	// OUR-CVFH estimator object 
	pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
	// Kd-Tree object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr ourcvfh_tree (new pcl::search::KdTree<pcl::PointXYZ>);

 	// Initialization of OUR-CVFH estimator object
	ourcvfh.setInputCloud (cluster);
	ourcvfh.setInputNormals (normals);
	ourcvfh.setSearchMethod (ourcvfh_tree);

	ourcvfh.setCurvatureThreshold (3.14/2);
	ourcvfh.setClusterTolerance (0.3);
	ourcvfh.setEPSAngleThreshold (0.78);
	ourcvfh.setMinPoints (30);
	
	ourcvfh.setNormalizeBins (false);
	ourcvfh.setAxisRatio (1);

	// OUR-CVFH descriptors determination
	ourcvfh.compute (*signature);

	if (signature->height != 1)
		signature->height = 1;
	if (signature->width != 1)
		signature->width = 1;
}