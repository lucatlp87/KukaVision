// CLASS <OURCVFHEstimation> DEFINITION
// The class <OURCVFHEstimation> contains all methods used in OUR-CVFH signature estimation.

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
	void CloudOURCVFHComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::VFHSignature308>::Ptr signature);
};