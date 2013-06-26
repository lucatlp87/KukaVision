// CLASS <Cloud> DEFINITION	
// In this file the <Cloud> class id defined. It contains variables defining needed clouds as private memebers and methods to get or set them.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Cloud
{
private:
	// Point cloud object
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	// Point cloud normals object
	pcl::PointCloud<pcl::Normal> point_cloud_normals;
	// Point cloud with normals object
	pcl::PointCloud<pcl::PointNormal> point_cloud_with_normals;
	// OUR-CVFH signature object
	pcl::PointCloud<pcl::VFHSignature308> point_cloud_OURCVFH;
	// OUR-CVFH signature vector
	std::vector<float> point_cloud_OURCVFH_vector;

	// Segmentation error
	bool seg_error;
	// No clusters
	bool no_clusters;

public:
	// **************************************************************************************************************************************************************
	// Constructor
	Cloud();
	// Destructor
	~Cloud();

	// I/O OPERATION ON PRIVATE MEMBERS *****************************************************************************************************************************
	// Get the point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
	// Get the normals cloud
	pcl::PointCloud<pcl::Normal>::Ptr GetNormals();
	// Get OUR-CVFH signature
	pcl::PointCloud<pcl::VFHSignature308>::Ptr GetOURCVFH();
	// Get OUR-CVFH vector
	std::vector<float> GetOURCVFHVector();
	// Get <seg_error> flag value
	bool GetSegError();
	// Get <no_clusters> flag value
	bool GetNoClusters();
	// Set the point cloud
	void SetCloud(pcl::PointCloud<pcl::PointXYZ> cloud_to_set);
	// Set the normals cloud
	void SetNormals(pcl::PointCloud<pcl::Normal> normals_to_set);
	// Set the point cloud with normals
	void SetCloudWithNormals(pcl::PointCloud<pcl::PointNormal> cloud_to_set);
	// Set the OUR-CVFH signature
	void SetOURCVFH(pcl::PointCloud<pcl::VFHSignature308> histogram_to_set);
	// Set the OUR-CVFH vector
	void SetOURCVFHVector();
	// Set <seg_error> flag value
	void SetSegError(bool flag_value);
	// Set <no_clusters> flag value
	void SetNoClusters(bool flag_value);
	// Load a cloud from disk
	void LoadCloud(std::string loading_path);
	// Load a OUR-CVFH signature from disk
	void LoadOURCVFH(std::string loading_path);
	// Save a cloud to disk
	std::string SaveCloud();
};
