// CLASS <SceneManagement> DEFINITION	
// In this file the <DBManagement> class id defined. It caontains methods used to perfrom Kd-Tree search in the DataBase

class SceneManagement
{
private:

	// Reference scene cloud object
	pcl::PointCloud<pcl::PointXYZ> ref_scene;
	// Vector containing the extracted clusters objects
  	std::vector<Cloud, Eigen::aligned_allocator<Cloud> > scene_objects_vector;

	// Default name of the file containing the list of paths corresponding to the DB subfolders containing correspondences with objects in the scene
	std::string default_name_file;
	// Actual subfolder path in boost format
	std::string sub_path;
	// Actual path to the file containing the point cloud file
	std::string actual_cloud_path;
	// Actual path to the file containing the name list
	std::string actual_name_path;

	// Vector containing SB subfolders containing correspondences with objects in the scene
	std::vector<std::string> subfolder_list;



public:
	// **************************************************************************************************************************************************************
	// Constructor
	SceneManagement();
	// Destructor
	~SceneManagement();

	// SCENE I/O ****************************************************************************************************************************************************
	// Get <ref_scene> attribute
	pcl::PointCloud<pcl::PointXYZ>::Ptr GetRefScene();
	// Set <ref_scene> atrribute
	void SetRefScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set);
	// Set scene object vector
	void SetSceneObjectVector(std::vector<Cloud, Eigen::aligned_allocator<Cloud> > vector_to_set);
	// Set <subfolder_list> attribute
	void SetSubfolderList(std::string subfolder_to_set);

	// Update the object
	void UpdateObject(std::string path_to_set);

	// Save a new scene
	void SaveScene();

	// VISUALIZATION ************************************************************************************************************************************************
	// Visualize the reference scene
	void VisualizeRefScene();
};
