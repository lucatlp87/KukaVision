// CLASS <OURCVFHEstimation> DEFINITION
// The class <OURCVFHEstimation> contains all methods used in OUR-CVFH signature estimation.

#include <Eigen/StdVector>

class OURCVFHEstimation
{
public:
	// **************************************************************************************************************************************************************
	// Constructor	
	OURCVFHEstimation();
	// Destructor
	~OURCVFHEstimation();

	// OUR-CVFH SIGNATURES ESTIMATION *******************************************************************************************************************************
	// Compute OUR-VFH signatures
	Eigen::Matrix<float, 4, 4> CloudOURCVFHComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals, 
												 pcl::PointCloud<pcl::VFHSignature308>::Ptr signature);
	// Transform the OUR-CVFH signature in a vector of <float>
	std::vector<float> OURCVFHSignatureTransformation (pcl::PointCloud<pcl::VFHSignature308>::Ptr signature);
};