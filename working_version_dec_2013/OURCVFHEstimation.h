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
	void CloudOURCVFHComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals, 
												 pcl::PointCloud<pcl::VFHSignature308>::Ptr signature,
												 std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > &transformation);
	// Transform the OUR-CVFH signature in a vector of <float>
	std::vector<float> OURCVFHSignatureTransformation (pcl::PointCloud<pcl::VFHSignature308>::Ptr signature);
};