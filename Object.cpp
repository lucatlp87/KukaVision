// <Object> CLASS METHODS IMPLEMENTATION

#include <Eigen/LU>

#include <iostream>

#include "Object.h"

// CONSTRUCTOR
Object::Object() {}
// DESTRUCTOR
Object::~Object() {}

// #################################################################################################################################################################
// I/O OPERATIONS ON PRIVATE MEMBERS ###############################################################################################################################
// #################################################################################################################################################################

// GETTING OBJECT CLOUD
pcl::PointCloud<pcl::PointXYZ>::Ptr
Object::GetObjectCloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	*output_cloud = object_cloud;
	return(output_cloud);
}
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
Eigen::Matrix<float, 4, 4>
Object::GetObjectRefPose() {return(object_ref_pose);}
// GETTING OBJECT POSE IN ACTUAL SCENE
Eigen::Matrix<float, 4, 4>
Object::GetObjectActualPose() {return(object_actual_pose);}
// GETTING OBJECT POSE TRANSFORMATION
Eigen::Matrix<float, 4, 4>
Object::GetObjectPoseTransformation() {return(object_pose_transformation);}

// SETTING THE OBJECT CLOUD
void
Object::SetObjectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	object_cloud = *cloud_to_set;
}
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
Object::SetObjectRefPose(Eigen::Matrix<float, 4, 4> matrix_to_set)
{
	for(size_t i = 0; i < 4; ++i)
		for(size_t j = 0; j < 4; ++j)
			object_ref_pose(i,j) = matrix_to_set(i,j);
}
// SETTING OBJECT POSE IN ACTUAL SCENE
void
Object::SetObjectActualPose(Eigen::Matrix<float, 4, 4> matrix_to_set)
{
	for(size_t i = 0; i < 4; ++i)
		for(size_t j = 0; j < 4; ++j)
			object_actual_pose(i,j) = matrix_to_set(i,j);
}
// SETTING OBJECT POSE TRANSFORMATION
void
Object::SetObjectPoseTransformation(bool in_out) 
{
	if (!in_out)
		// Determination of the transformation matrix for an inner object
		object_pose_transformation = object_actual_pose*object_ref_pose.inverse();
	else
	{
		// Final pose matrix
		Eigen::Matrix<float, 4, 4> out_of_the_table_pose;
		
		// The orientation of the final pose is the same of the actual one (no movements are required for the Kuka)
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 3; ++j)
				if (i == 3)
					out_of_the_table_pose(i,j) = 0;
				else
					out_of_the_table_pose(i,j) = object_actual_pose(i,j);

		// The position is set in order to keep the object fall down out of the tabletop
		out_of_the_table_pose(0,3) = 0.5;
		out_of_the_table_pose(1,3) = 1;
		out_of_the_table_pose(2,3) = 0.5;
		out_of_the_table_pose(3,3) = 1;

		// Determination of the transformation matrix for an outer object
		object_pose_transformation = object_actual_pose*out_of_the_table_pose.inverse();
	}
}
