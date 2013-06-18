// <SceneManagement> CLASS METHODS IMPLEMENTATION

#include <ctime>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Cloud.h"
#include "SceneManagement.h"

// CONSTRUCTOR
SceneManagement::SceneManagement() 
{
	boost::filesystem::path scene_base_dir = "../SceneDB/";
	if (!boost::filesystem::exists (scene_base_dir) && !boost::filesystem::is_directory (scene_base_dir))
		boost::filesystem::create_directory(scene_base_dir);

	default_name_file = "SceneObjectsList.list";
}

// DESTRUCTOR
SceneManagement::~SceneManagement() {}

// #################################################################################################################################################################
// I/O OPERATIONS ON PRIVATE MEMBERS ###############################################################################################################################
// #################################################################################################################################################################

// GETTING <ref_scene> ATTRIBUTE ***********************************************************************************************************************************
pcl::PointCloud<pcl::PointXYZ>::Ptr 
SceneManagement::GetRefScene()
{
	// The function return a pointer to the <point_cloud> variable. If no point cloud is stored in <point_cloud> a warning is printed out. 
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_pointer = ref_scene;

	if (cloud_pointer == NULL)
		pcl::console::print_error ("\t[WARNING] The cloud is empty!\n\n");
				
	return (cloud_pointer);
}

// SETTING <ref_scene> ATTRIBUTE ************************************************************************************************************************************
void
SceneManagement::SetRefScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	ref_scene = *cloud_to_set;
}

// SETTING <scene_objects_vector> ATTRIBUTE *************************************************************************************************************************
void
SceneManagement::SetSceneObjectVector(std::vector<Cloud, Eigen::aligned_allocator<Cloud> > vector_to_set)
{
	scene_objects_vector.resize(vector_to_set.size());
	scene_objects_vector = vector_to_set;
}

// SETTING <subfolder_list> ATTRIBUTE *******************************************************************************************************************************
void
SceneManagement::SetSubfolderList(std::string subfolder_to_set)
{
	// The function appends the subfolder in input to the <subfolder_list> attribute

	subfolder_list.push_back(subfolder_to_set);
}

// UPDATING THE OBJECT **********************************************************************************************************************************************
void
SceneManagement::UpdateObject(std::string path_to_set)
{
	std::string base_dir = "../SceneDB/";

	base_dir.append(path_to_set);
	sub_path = base_dir.append("/");

	path_to_set.append(".pcd");
	actual_cloud_path = base_dir.append(path_to_set);
	actual_name_path = base_dir.append(default_name_file);
}


// SAVING THE SCENE
void
SceneManagement::SaveScene()
{
	// Name of the actual scene
	std::string scene_name;
	// Pointer of the file containing paths to DB subfolders corresponding to objects in the actual scene
  	std::ofstream name_file_ptr;
  	// Pointer to the reference scene
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_scene_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  	// User choice parsing
    std::cout << std::endl << "\tThe actual scene contains " << subfolder_list.size() << " objects" << std::endl;
	std::cout << "\tPlease insert the name of the scene you are saving: " << std::flush;
	std::cin >> scene_name;
	
	// Update the object with the current DB subfolder
	this->UpdateObject(scene_name);

	// Create the SceneDB subfolder if it does not exixts yet
	if (!boost::filesystem::exists (sub_path) && !boost::filesystem::is_directory (sub_path))
		boost::filesystem::create_directory(sub_path);
	
	// Saving the cloud object
	pcl::console::print_error ("\n\tSaving the cloud...");
	pcl::io::savePCDFile(actual_cloud_path,ref_scene,false);
	pcl::console::print_error ("\t\t\tdone\n");

	// Saving the <SceneObjectList> file
	pcl::console::print_error ("\tSaving the name list...");
	name_file_ptr.open (actual_name_path.c_str());

    // File update (new paths are added in append mode)
    for (size_t i = 0; i < subfolder_list.size (); ++i)
     	name_file_ptr << i+1 << " " << subfolder_list[i] << "\n";
    
    // Close the file
    name_file_ptr.close ();
    pcl::console::print_error ("\t\t\tdone\n");

    pcl::console::print_error ("\tSaving transformation matrices...");
    pcl::console::print_error ("\tdone\n");
}

// #################################################################################################################################################################
// VISUALIZATION ###################################################################################################################################################
// #################################################################################################################################################################

// VISUALIZING THE REFERENCE SCENE
void
SceneManagement::VisualizeRefScene()
{
	// Reference scene cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	// Visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));         
    // Viewport id
    int viewport = 0;

    // Visualizaton object initialization
    scene_viewer->initCameraParameters ();
    scene_viewer->addCoordinateSystem (1.0);
    scene_viewer->setBackgroundColor (0, 0, 0);

    // Adding the whole Kinect acquisition (white)
    ref_cloud_ptr = this->GetRefScene();
    scene_viewer->createViewPort (0,0,0.5,1,viewport);
    scene_viewer->addPointCloud<pcl::PointXYZ> (ref_cloud_ptr, "Whole_acquisition",viewport);
	
	double viewport_pos = 1;

    for (size_t i = 0; i < subfolder_list.size(); ++i)
    {
    	// Object cloud pointer
    	pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointer (new pcl::PointCloud<pcl::PointXYZ>);
    	// Object identifier in the visualizer
    	std::stringstream object_name;
    	object_name << "object_" << i+1;

		// Generating color elements
    	std::vector<int> color_vector;
    	color_vector.resize(3);
   		// Initialize the random seed
    	srand(time(NULL)*2112*i);

    	for (size_t color_idx = 0; color_idx < 3; ++color_idx)
    	{
    		// Get a random number between 0 and 255
    		color_vector[color_idx] = rand() % 256;
    	}

		// Loading the object cluster
		object_pointer = scene_objects_vector[i].GetCloud();

		// Calculating the centroid
		Eigen::Vector4f centroid;
    	pcl::compute3DCentroid (*object_pointer, centroid);

		// Setting other visualizer settings
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color (object_pointer, color_vector[1], color_vector[2], color_vector[3]);
		scene_viewer->addPointCloud<pcl::PointXYZ> (object_pointer, cluster_color, object_name.str());

		// Update viewport id
		// if (viewport == )
		// scene_viewer->createViewPort(0.5,min_y,1,max_y,viewport);
		// viewport_pos = viewport_pos+1;
	}

    // Viewer loop
    while (!scene_viewer->wasStopped ())
    { 
        scene_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    scene_viewer->close();
}