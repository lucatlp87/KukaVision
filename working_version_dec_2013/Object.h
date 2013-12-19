// CLASS <Object> DEFINITION	
// In this file the <Object> class id defined.

#include <Eigen/StdVector>
#include <flann/flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Object
{
private:
	// Point cloud corresponding to object
	pcl::PointCloud<pcl::PointXYZ> object_cloud;
	// Object ID
	int object_id;
	// Object type
	std::string object_type;
	// Object model subfolder path
	std::string object_model_path;

	// Object pose in the reference scene
	Eigen::Matrix4f object_ref_pose;
	// Object pose in the actual scene
	Eigen::Matrix4f object_actual_pose;
	// Object pose transformation
	Eigen::Matrix4f object_pose_transformation;

public:
	// **************************************************************************************************************************************************************
	// Constructor
	Object();
	// Destructor
	~Object();

	// PRIVATE MEMEBERS I/O ****************************************************************************************************************************************************
	// Get the object cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr GetObjectCloud();
	// Get the object ID
	int GetObjectID();
	// Get object type
	std::string GetObjectType();
	// Get object model path
	std::string GetObjectModelPath();
	// Get object pose in reference scene
	Eigen::Matrix4f GetObjectRefPose();
	// Get object pose in actual scene
	Eigen::Matrix4f GetObjectActualPose();
	// Get object pose transformation
	Eigen::Matrix4f GetObjectPoseTransformation();

	// Set the object cloud
	void SetObjectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set);
	// Set the object ID
	void SetObjectID(int id_to_set);
	// Set object type
	void SetObjectType(std::string type_to_set);
	// Set object model path
	void SetObjectModelPath(std::string path_to_set);
	// Set object pose in reference scene
	void SetObjectRefPose(Eigen::Matrix4f matrix_to_set);
	// Set object pose in actual scene
	void SetObjectActualPose(Eigen::Matrix4f matrix_to_set);
	// Set object pose transformation
	void SetObjectPoseTransformation(bool in_out);
};