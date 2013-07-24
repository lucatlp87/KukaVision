// CLASS <CloudProcessing> DEFINITION
// The class <CloudProcessing> contains all methods used in cloud filtering and clusters extraction.

class CloudProcessing
{
public:
	// **************************************************************************************************************************************************************
	// Constructor	
	CloudProcessing();
	// Destructor
	~CloudProcessing();


	// CLOUD FILTERS ************************************************************************************************************************************************
	// Pass-through filter
	void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	// Voxel grid filter
	void MLSFilterAndNormalsComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals,
										pcl::PointCloud<pcl::PointNormal>::Ptr cluster_with_normals);

	// CLOUD STORAGE ************************************************************************************************************************************************
	// Listing ObjectDB items
	void ListDBItems();
	// Saving the cloud model in object models DB
	std::string SaveCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cluster_with_normals, pcl::PointCloud<pcl::VFHSignature308>::Ptr OURCVFH_signature);
};