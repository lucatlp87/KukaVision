// CLASS <Object> DEFINITION	
// In this file the <Object> class id defined.

#include <flann/flann.h>

class Object
{
private:
	// Object ID
	int object_id;
	// Object type
	std::string object_type;
	// Object model subfolder path
	std::string object_model_path;

	// Object pose in the reference scene
	flann::Matrix<float> object_ref_pose;
	// Object pose in the actual scene
	flann::Matrix<float> object_actual_pose;
	// Object pose transformation
	flann::Matrix<float> object_pose_transformation;

public:
	// **************************************************************************************************************************************************************
	// Constructor
	Object();
	// Destructor
	~Object();

	// PRIVATE MEMEBERS I/O ****************************************************************************************************************************************************
	// Get the object ID
	int GetObjectID();
	// Get object type
	std::string GetObjectType();
	// Get object model path
	std::string GetObjectModelPath();
	// Get object pose in reference scene
	flann::Matrix<float> GetObjectRefPose();
	// Get object pose in actual scene
	flann::Matrix<float> GetObjectActualPose();
	// Get object pose transformation
	flann::Matrix<float> GetObjectPoseTransformation();

	// Set the object ID
	void SetObjectID(int id_to_set);
	// Set object type
	void SetObjectType(std::string type_to_set);
	// Set object model path
	void SetObjectModelPath(std::string path_to_set);
	// Set object pose in reference scene
	void SetObjectRefPose(flann::Matrix<float> matrix_to_set);
	// Set object pose in actual scene
	void SetObjectActualPose(flann::Matrix<float> matrix_to_set);
	// Set object pose transformation
	void SetObjectPoseTransformation(flann::Matrix<float> matrix_to_set);
};