// CLASS <SceneManagement> DEFINITION	
// In this file the <DBManagement> class id defined. It caontains methods used to perfrom Kd-Tree search in the DataBase
#include "Object.h"

class SceneManagement
{
private:

	typedef std::pair<std::string, int> ref_pair;

	// Reference scene cloud object
	pcl::PointCloud<pcl::PointXYZ> ref_scene;
	// Actual scene cloud object
	pcl::PointCloud<pcl::PointXYZ> actual_scene;

	// Default name of the file containing the list of paths corresponding to the DB subfolders containing correspondences with objects in the scene
	std::string default_name_file;
	// Actual subfolder path in boost format
	std::string sub_path;
	// Actual path to the file containing the point cloud file
	std::string actual_cloud_path;
	// Actual path to the file containing the name list
	std::string actual_name_path;

	// Vector containing all elements of the reference scene in <Object> format
	std::vector<Object, Eigen::aligned_allocator<Object> > ref_objects_list;
	// Vector containing inner elements of the actual scene in <Object> format
	std::vector<Object, Eigen::aligned_allocator<Object> > in_objects_list;
	// Vector containing outer elements of the actual scene in <Object> format
	std::vector<Object, Eigen::aligned_allocator<Object> > out_objects_list;

public:
	// **************************************************************************************************************************************************************
	// Constructor
	SceneManagement();
	// Destructor
	~SceneManagement();

	// SCENE I/O ****************************************************************************************************************************************************
	// Get <ref_scene> attribute
	pcl::PointCloud<pcl::PointXYZ>::Ptr GetRefScene();
	// Get object model subfolder list
	std::vector<ref_pair> GetModelSubfolderList();
	// Get <in_objects_list> attribute
	std::vector<Object, Eigen::aligned_allocator<Object> > GetInObjectsVector();
	// Get <out_objects_vector> attribute
	std::vector<Object, Eigen::aligned_allocator<Object> > GetOutObjectsVector();
	
	// Set <ref_scene> atrribute
	void SetRefScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set);
	// Insert a <Object> in the vector <object_list>
	void InsertObject(Object object_to_insert);
	// Update the object
	void UpdateObject(std::string scene_name);
	// Update the inner objects vector
	void UpdateInnerObjectsVector(int index_to_search);
	// Update the outer objects vector
	// void UpdateOuterObjectsVector(Cloud object_to_insert, int obj_idx);

	// Save a new scene
	void SaveScene();
	// Load a scene from DB
	void LoadRefScene(std::string scene_to_upload);

	// VISUALIZATION ************************************************************************************************************************************************
	// Visualize the reference scene
	void VisualizeRefScene(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector);
};
