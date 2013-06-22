#include <iostream>

#include "Object.h"

// CONSTRUCTOR
Object::Object() 
{
	// Allocate pose matrices as identity rotation and null translation
	object_ref_pose = flann::Matrix<float> (new float[16],4,4);
	object_actual_pose = flann::Matrix<float> (new float[16],4,4);
	object_pose_transformation = flann::Matrix<float> (new float[16],4,4);

}
// DESTRUCTOR
Object::~Object() {}

// #################################################################################################################################################################
// I/O OPERATIONS ON PRIVATE MEMBERS ###############################################################################################################################
// #################################################################################################################################################################

// GETTING OBJECT ID
int
Object::GetObjectID() {return(object_id);}
// GETTING OBJECT TYPE
std::string
Object::GetObjectType() {return(object_type);}
// GETTING OBJECT MODEL PATH
std::string
Object::GetObjectModelPath() {return(object_model_path);}
// GETTING OBJECT POSE IN REFERENCE SCENE
flann::Matrix<float>
Object::GetObjectRefPose() {return(object_ref_pose);}
// GETTING OBJECT POSE IN ACTUAL SCENE
flann::Matrix<float>
Object::GetObjectActualPose() {return(object_actual_pose);}
// GETTING OBJECT POSE TRANSFORMATION
flann::Matrix<float>
Object::GetObjectPoseTransformation() {return(object_pose_transformation);}

// SETTING THE OBJECT ID 
void
Object::SetObjectID(int id_to_set) {object_id = id_to_set;}
// SETTING OBJECT TYPE
void
Object::SetObjectType(std::string type_to_set) {object_type = type_to_set;}
// SETTING OBJECT MODEL PATH
void
Object::SetObjectModelPath(std::string path_to_set) {object_model_path = path_to_set;}
// SETTING OBJECT POSE IN REFERENCE SCENE
void
Object::SetObjectRefPose(flann::Matrix<float> matrix_to_set)
{
	for(size_t i = 0; i < 4; ++i)
		for(size_t j = 0; j < 4; ++j)
			object_ref_pose[i][j] = matrix_to_set[i][j];
}
// SETTING OBJECT POSE IN ACTUAL SCENE
void
Object::SetObjectActualPose(flann::Matrix<float> matrix_to_set)
{
	for(size_t i = 0; i < 4; ++i)
		for(size_t j = 0; j < 4; ++j)
			object_actual_pose[i][j] = matrix_to_set[i][j];
}
// SETTING OBJECT POSE TRANSFORMATION
void
Object::SetObjectPoseTransformation(flann::Matrix<float> matrix_to_set) 
{
	for(size_t i = 0; i < 4; ++i)
		for(size_t j = 0; j < 4; ++j)
			object_pose_transformation[i][j] = matrix_to_set[i][j];
}
