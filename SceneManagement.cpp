// <SceneManagement> CLASS METHODS IMPLEMENTATION

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Cloud.h"
#include "SceneManagement.h"

typedef std::pair<std::string, int> ref_pair;

// CONSTRUCTOR
SceneManagement::SceneManagement() 
{
	boost::filesystem::path scene_base_dir = "../SceneDB/";
	if (!boost::filesystem::exists (scene_base_dir) && !boost::filesystem::is_directory (scene_base_dir))
		boost::filesystem::create_directory(scene_base_dir);

	default_name_file = "SceneObjectsList.txt";
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

// GETTING OBJECT MODEL SUBFOLDER LIST
std::vector<ref_pair>
SceneManagement::GetModelSubfolderList()
{
	// Return vector
	std::vector<ref_pair> return_vector;

	// Explore the <Object> type vector <ref_objects_list>
	for (size_t list_idx = 0; list_idx < ref_objects_list.size(); ++list_idx)
	{
		ref_pair actual_element;
		actual_element.first = ref_objects_list[list_idx].GetObjectModelPath();
		actual_element.second = ref_objects_list[list_idx].GetObjectID();
		return_vector.push_back(actual_element);
	}

	return (return_vector);
}

// GETTING <in_objects_vector> ATTRIBUTE ****************************************************************************************************************************
std::vector<Object> 
SceneManagement::GetInObjectsVector()
{
	return (in_objects_list);
}

// GETTING <otu_objects_vector> ATTRIBUTE ***************************************************************************************************************************
std::vector<Object> 
SceneManagement::GetOutObjectsVector()
{
	return (out_objects_list);
}

// SETTING <ref_scene> ATTRIBUTE ************************************************************************************************************************************
void
SceneManagement::SetRefScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	ref_scene = *cloud_to_set;
}


// INSERTING AN OBJECT
void
SceneManagement::InsertObject(Object object_to_insert)
{
	ref_objects_list.push_back(object_to_insert);
}

// UPDATING THE OBJECT **********************************************************************************************************************************************
void
SceneManagement::UpdateObject(std::string scene_name)
{
	// Scene DB bas_dir
	std::string db_base_dir = "../SceneDB/";
	// Updating the path to the subfolder of scenes DB
	sub_path = db_base_dir.append(scene_name);
	sub_path.append("/");
	// Updating the path to name file
	actual_name_path = sub_path;
	actual_name_path.append(default_name_file);
	// Updating the path to cloud file
	actual_cloud_path = sub_path;
	actual_cloud_path.append(scene_name).append(".pcd");
}

// UPDATING THE INNER OBJECTS VECTOR
void
SceneManagement::UpdateInnerObjectsVector(int index_to_search)
{
	for (size_t search_idx = 0; search_idx < ref_objects_list.size(); ++search_idx)
	{
		if (ref_objects_list[search_idx].GetObjectID() == index_to_search)
		{
			in_objects_list.push_back(ref_objects_list[search_idx]);
			break;
		}
	}
}

// UPDATING THE OUTER OBJECTS VECTOR
void
SceneManagement::UpdateOuterObjectsVector(Cloud object_to_insert, int obj_idx)
{
	Object new_object;
	new_object.SetObjectID(obj_idx);
	new_object.SetObjectType("Unknown");
	new_object.SetObjectModelPath("Unknown");

	out_objects_list.push_back(new_object);
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
    std::cout << std::endl << "\tThe actual scene contains " << ref_objects_list.size() << " objects" << std::endl;
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
    for (size_t i = 0; i < ref_objects_list.size (); ++i)
     	name_file_ptr << ref_objects_list[i].GetObjectID() 
     				  << "\n" << ref_objects_list[i].GetObjectType()
     				  << "\n" << ref_objects_list[i].GetObjectModelPath() << "\n";
    
    // Close the file
    name_file_ptr.close ();
    pcl::console::print_error ("\t\t\tdone\n");

    // Saving the pose of the object
    pcl::console::print_error ("\tSaving reference pose matrix...");
    for (size_t i=0; i < ref_objects_list.size(); ++i)
    {	

	   	std::string flann_path = sub_path;
    	flann_path.append("object_").append(boost::lexical_cast<std::string>(ref_objects_list[i].GetObjectID())).append("_ref_pose.h5");
		
    	flann::save_to_file (ref_objects_list[i].GetObjectRefPose(),flann_path,("obj_reference_pose"));
    }
    pcl::console::print_error ("\tdone\n");
}

// LOADING A SCENE FROM DB
void
SceneManagement::LoadRefScene(std::string scene_to_upload)
{
	// Updating the object
	this->UpdateObject(scene_to_upload);
	// Loading the cloud
	pcl::io::loadPCDFile(actual_cloud_path,ref_scene);
	
	// Loading file names in the <subfolder_list> attribute
	std::ifstream name_file_ptr;
	name_file_ptr.open (actual_name_path.c_str());

	while (!name_file_ptr.eof ())
	{
		Object read_object;
		std::string line;
		flann::Matrix<float> read_pose (new float[16],4,4);

		// Get the first line of each block (objet id)
		getline (name_file_ptr, line);
		if (line.empty ())
	  		continue;
	  	// Cast of the string to int
	  	int id_number = boost::lexical_cast<int>(line.c_str());
		read_object.SetObjectID(id_number);
		// Get the second line of each block (object type)
		getline (name_file_ptr, line);
		read_object.SetObjectType(line);
		// Get the third line of each block (object model path)
		getline (name_file_ptr, line);
		read_object.SetObjectModelPath(line);

    	std::string flann_path = sub_path;
    	flann_path.append("object_").append(boost::lexical_cast<std::string>(id_number)).append("_ref_pose.h5");

    	std::cout << std::endl << flann_path << std::endl;

		flann::load_from_file(read_pose,flann_path,"obj_reference_pose");
		read_object.SetObjectRefPose(read_pose);

	  	ref_objects_list.push_back(read_object);
	}
	name_file_ptr.close ();
}


// #################################################################################################################################################################
// VISUALIZATION ###################################################################################################################################################
// #################################################################################################################################################################

// VISUALIZING THE REFERENCE SCENE
void
SceneManagement::VisualizeRefScene(std::vector<Cloud, Eigen::aligned_allocator<Cloud> > clusters_vector)
{
	// Reference scene cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	// Visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));         
    // Viewport id
    int viewport = 1;
    // Viewports size
    double viewport_step = 1/(double)clusters_vector.size();

    // Visualizaton object initialization
    scene_viewer->initCameraParameters ();
    scene_viewer->addCoordinateSystem (1.0);
    scene_viewer->setBackgroundColor (0, 0, 0,viewport);
    scene_viewer->setWindowBorders(true);

    // Adding the whole Kinect acquisition (white)
    ref_cloud_ptr = this->GetRefScene();
    scene_viewer->createViewPort (0,viewport_step,1,1,viewport);
    scene_viewer->addPointCloud<pcl::PointXYZ> (ref_cloud_ptr, "Whole_acquisition",viewport);
    scene_viewer->resetCameraViewpoint("Whole_acquisition");
    scene_viewer->addText("ACQUIRED SCENE CLOUD", 10, 10, 1, 0, 0, "AcquiredCloudText", viewport);
	scene_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText", viewport);
	
    for (double i = 0; i < clusters_vector.size(); ++i)
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
    		// Get a random number between 0 and 255
    		color_vector[color_idx] = rand() % 256;

		// Loading the object cluster
		object_pointer = clusters_vector[i].GetCloud();

		// Setting other visualizer settings and adding the cluster in his viewport
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color (object_pointer, color_vector[1], color_vector[2], color_vector[3]);

		scene_viewer->createViewPort(i*viewport_step,0,(i+1)*viewport_step,viewport_step,viewport);
		scene_viewer->addPointCloud<pcl::PointXYZ> (object_pointer, cluster_color, object_name.str().append("_info"), viewport);
        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, object_name.str().append("_info"));
		if (((int)i)%2 == 0)
			scene_viewer->setBackgroundColor(0.1,0.1,0.1,viewport+1);
		else
			scene_viewer->setBackgroundColor(0.15,0.15,0.15,viewport+1);
	    scene_viewer->resetCameraViewpoint(object_name.str().append("_info"));

	    flann::Matrix<float> object_pose (new float[16],4,4);
	    object_pose = ref_objects_list[i].GetObjectRefPose();
	    std::string object_pose_string;
	    for(size_t matrix_i = 0; matrix_i < 4; ++matrix_i)
	    	for(size_t matrix_j = 0; matrix_j < 4; ++matrix_j)
	    	{
	    		object_pose_string.append(boost::lexical_cast<std::string>((int)object_pose[matrix_i][matrix_j]));
	    		if(matrix_j == 3 && i < 3)
	    			object_pose_string.append("\n         ");
	    		else
	    			object_pose_string.append(" ");
	    	}

		std::stringstream object_info;
		object_info << object_name.str() << "\nType: " << ref_objects_list[i].GetObjectType() << "\nPose: " << object_pose_string;
		scene_viewer->addText(object_info.str(), 10, 10, 0.8, 0.2, 0.2, object_name.str(), viewport);
	    scene_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 10, object_name.str(), viewport);


	    // Adding the cluster to the main cloud
	    scene_viewer->addPointCloud<pcl::PointXYZ> (object_pointer,cluster_color,object_name.str().append("_main"), 1);
        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, object_name.str().append("_main"));
	}

    // Viewer loop
    while (!scene_viewer->wasStopped ())
    { 
        scene_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    scene_viewer->close();
}

