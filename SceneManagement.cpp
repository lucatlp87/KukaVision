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

// GETTING <actual_scene> ATTRIBUTE ***********************************************************************************************************************************
pcl::PointCloud<pcl::PointXYZ>::Ptr 
SceneManagement::GetActualScene()
{
	// The function return a pointer to the <point_cloud> variable. If no point cloud is stored in <point_cloud> a warning is printed out. 
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_pointer = actual_scene;

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
std::vector<Object, Eigen::aligned_allocator<Object> > 
SceneManagement::GetInObjectsVector()
{
	return (in_objects_list);
}

// GETTING <otu_objects_vector> ATTRIBUTE ***************************************************************************************************************************
std::vector<Object, Eigen::aligned_allocator<Object> > 
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

// SETTING <actual_scene> ATTRIBUTE *********************************************************************************************************************************
void
SceneManagement::SetActualScene(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	actual_scene = *cloud_to_set;
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

// UPDATING THE INNER OBJECTS VECTOR ********************************************************************************************************************************
void
SceneManagement::UpdateInnerObjectsVector(int index_to_search, Eigen::Matrix<float, 4, 4> pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	for (size_t search_idx = 0; search_idx < ref_objects_list.size(); ++search_idx)
	{
		if (ref_objects_list[search_idx].GetObjectID() == index_to_search)
		{
			// Storing the object cloud
			ref_objects_list[search_idx].SetObjectCloud(cloud_to_set);
			// Storing the actual pose
			ref_objects_list[search_idx].SetObjectActualPose(pose);

			// Calculating the transformation matrix
			pcl::console::print_error("\n\t\tCalculating the transformation matrix between poses...");
			ref_objects_list[search_idx].SetObjectPoseTransformation(0);
			pcl::console::print_error("\tdone\n");

			// Updating the inner objects list
			in_objects_list.push_back(ref_objects_list[search_idx]);
			break;
		}
	}
}

// UPDATING THE OUTER OBJECTS VECTOR
void
SceneManagement::UpdateOuterObjectsVector(int index_to_search, Eigen::Matrix<float, 4, 4> pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_set)
{
	// Local object
	Object outer_object;

	// Filling the local object
	outer_object.SetObjectCloud(cloud_to_set);
	outer_object.SetObjectID(index_to_search);
	outer_object.SetObjectType("Unkenown");
	outer_object.SetObjectModelPath("Unkenown");
	outer_object.SetObjectActualPose(pose);

	// Calculating the transformation matrix
	pcl::console::print_error("\n\t\tCalculating the transformation matrix between poses...");
	outer_object.SetObjectPoseTransformation(1);
	pcl::console::print_error("\tdone\n");

	// Updating the inner objects list
	out_objects_list.push_back(outer_object);
}

// LISTING DB ITEMS *************************************************************************************************************************************************
void
SceneManagement::ListDBItems()
{
	// Path to the DB directory
  	boost::filesystem::path ObjectDB_path = "../SceneDB";
  	// Iterator
  	int folder_number = 0;

  	for (boost::filesystem::directory_iterator it (ObjectDB_path); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
          	std::stringstream folder_name;
            folder_name << it->path ().c_str();
            std::cout << "\t\t" << folder_name.str().substr(folder_name.str().find_last_of("/")+1) << std::endl;
            ++folder_number;
        }
    }

    if (folder_number == 0)
      std::cout << "\t\tNo scenes are stored in the DB!" << std::endl;
}

// SAVING THE SCENE *************************************************************************************************************************************************
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

    // Listing available models
  	std::cout << "\tAvailable scenes are:" << std::endl << std::endl;
  	this->ListDBItems();
  	std::cout << std::endl << "\t(if this is a new scene, please insert a new folder: it will be automatically created in the DB)" << std::endl;

	std::cout << "\tPlease insert the name of the scene you are saving: " << std::flush;
	std::cin >> scene_name;
	
	// Update the object with the current DB subfolder
	this->UpdateObject(scene_name);

	// Create the SceneDB subfolder if it does not exixts yet
	if (!boost::filesystem::exists (sub_path) && !boost::filesystem::is_directory (sub_path))
		boost::filesystem::create_directory(sub_path);
	
	// Saving the cloud object
	pcl::console::print_error ("\n\tSaving the reference scene cloud...");
	pcl::io::savePCDFile(actual_cloud_path,ref_scene,false);
	pcl::console::print_error ("\t\t\tdone\n");

	// Saving the <SceneObjectList> file
	pcl::console::print_error ("\tSaving the objects infos...");
	name_file_ptr.open (actual_name_path.c_str());

    // File update (new paths are added in append mode)
    for (size_t i = 0; i < ref_objects_list.size (); ++i)
     	name_file_ptr << ref_objects_list[i].GetObjectID() 
     				  << "\n" << ref_objects_list[i].GetObjectType()
     				  << "\n" << ref_objects_list[i].GetObjectModelPath() 
     				  << "\n" << ref_objects_list[i].GetObjectRefPose() << "\n";
    
    // Close the file
    name_file_ptr.close ();
    pcl::console::print_error ("\t\tdone\n");
}

// LOADING A SCENE FROM DB ******************************************************************************************************************************************
bool
SceneManagement::LoadRefScene(std::string scene_to_upload)
{
	// Cluster pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize (new pcl::PointCloud<pcl::PointXYZ>);
	// Visualization object
	boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));     
	// User choice
	std::string continue_or_stop;

	// Updating the object
	this->UpdateObject(scene_to_upload);
	// Loading the cloud
	pcl::io::loadPCDFile(actual_cloud_path,ref_scene);
	cloud_to_visualize = this->GetRefScene();

	// Showing the chosen scene
	std::cout << "\n\tVisualizing the chosen scene... (please quit the visualizer to continue)" << std::endl;
    scene_viewer->initCameraParameters ();
	scene_viewer->addCoordinateSystem (1.0);
	scene_viewer->setBackgroundColor (0, 0, 0);
	scene_viewer->setWindowBorders(true);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_midori (cloud_to_visualize, 227, 249, 136);

    scene_viewer->addPointCloud<pcl::PointXYZ> (cloud_to_visualize, color_midori, "Scene");
    scene_viewer->resetCameraViewpoint("Scene");
    scene_viewer->addText("CHOSEN REFERENCE SCENE", 10, 10, 1, 0, 0, "SceneText");
    scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Scene");
	scene_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "SceneText");

    while (!scene_viewer->wasStopped ())
    { 
        scene_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    scene_viewer->close();

	std::cout << "\tContinue? " << std::flush;
    std::cin >> continue_or_stop;
    
    while (true)
    {
    	if(!continue_or_stop.compare("y") || !continue_or_stop.compare("n"))
    		break;
    	else
    	{
    		pcl::console::print_error("\t[WARNING] Please insert ""y"" or ""n"" ");
    		std::cin >> continue_or_stop;
    	}
    }

    if (!continue_or_stop.compare("y"))
    {
		// Loading file names in the <subfolder_list> attribute
		std::ifstream name_file_ptr;
		name_file_ptr.open (actual_name_path.c_str());

		while (!name_file_ptr.eof ())
		{
			Object read_object;
			std::string line;
			float matrix_element;
			Eigen::Matrix<float, 4, 4> read_pose;

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
			// Get matrices elements
			for (int matrix_row = 0; matrix_row < 4; ++matrix_row)
				for (int matrix_col = 0; matrix_col < 4; ++matrix_col)
				{
					name_file_ptr >> matrix_element;
					read_pose(matrix_row,matrix_col) = matrix_element;
				}
			read_object.SetObjectRefPose(read_pose);

		  	ref_objects_list.push_back(read_object);
		}
		name_file_ptr.close ();
		return(0);
	}
	else
		return(1);
}


// #################################################################################################################################################################
// VISUALIZATION ###################################################################################################################################################
// #################################################################################################################################################################

// PRINTING OUTER AND INNER OBJECTS INFOS **************************************************************************************************************************
void
SceneManagement::ShowInfo()
{
	// Local matrix
	Eigen::Matrix<float, 4, 4> matrix_to_print;
	
	// Printing infos about inner objects
	pcl::console::print_error ("\n\t\tINNER OBJECTS\n");
	std::cout << "\t\t(" << in_objects_list.size() << " elements)" << std::endl;

	if (in_objects_list.size() > 0)
		for (int in_idx = 0; in_idx < in_objects_list.size(); ++in_idx)
		{

			std::cout << "\n\t\tOBJECT " << in_idx << " --------------------------------------------------------------------------------------------------------------------" << std::endl;
			// Object type
			std::cout << "\t\tType -> " << in_objects_list[in_idx].GetObjectType() << std::endl << std::endl;
			// Reference pose
			matrix_to_print = in_objects_list[in_idx].GetObjectRefPose();
			std::cout << "\t\t\t\t\t|" << matrix_to_print(0,0) << "\t\t" << matrix_to_print(0,1) << "\t\t" << matrix_to_print(0,2) << "\t\t" << matrix_to_print(0,3) << "|\n"
					  << "\t\tReference pose ->"
					  << "\t|" << matrix_to_print(1,0) << "\t\t" << matrix_to_print(1,1) << "\t\t" << matrix_to_print(1,2) << "\t\t" << matrix_to_print(1,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(2,0) << "\t\t" << matrix_to_print(2,1) << "\t\t" << matrix_to_print(2,2) << "\t\t" << matrix_to_print(2,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(3,0) << "\t\t\t" << matrix_to_print(3,1) << "\t\t\t" << matrix_to_print(3,2) << "\t\t\t" << matrix_to_print(3,3) << "|\n\n";
			// Actual pose
			matrix_to_print = in_objects_list[in_idx].GetObjectActualPose();
			std::cout << "\t\t\t\t\t|" << matrix_to_print(0,0) << "\t\t" << matrix_to_print(0,1) << "\t\t" << matrix_to_print(0,2) << "\t\t" << matrix_to_print(0,3) << "|\n"
					  << "\t\tActual pose ->"
					  << "\t\t|" << matrix_to_print(1,0) << "\t\t" << matrix_to_print(1,1) << "\t\t" << matrix_to_print(1,2) << "\t\t" << matrix_to_print(1,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(2,0) << "\t\t" << matrix_to_print(2,1) << "\t\t" << matrix_to_print(2,2) << "\t\t" << matrix_to_print(2,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(3,0) << "\t\t\t" << matrix_to_print(3,1) << "\t\t\t" << matrix_to_print(3,2) << "\t\t\t" << matrix_to_print(3,3) << "|\n\n";	
			// Transformation matrix
			matrix_to_print = in_objects_list[in_idx].GetObjectPoseTransformation();
			std::cout << "\t\t\t\t\t|" << matrix_to_print(0,0) << "\t\t" << matrix_to_print(0,1) << "\t\t" << matrix_to_print(0,2) << "\t\t" << matrix_to_print(0,3) << "|\n"
					  << "\t\tPose transformation ->"
					  << "\t|" << matrix_to_print(1,0) << "\t\t"<< matrix_to_print(1,1) << "\t\t" << matrix_to_print(1,2) << "\t\t" << matrix_to_print(1,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(2,0) << "\t\t" << matrix_to_print(2,1) << "\t\t" << matrix_to_print(2,2) << "\t\t" << matrix_to_print(2,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(3,0) << "\t\t\t" << matrix_to_print(3,1) << "\t\t\t" << matrix_to_print(3,2) << "\t\t\t" << matrix_to_print(3,3) << "|\n\n";	
		}

	// Printing infos about inner objects
	pcl::console::print_error ("\n\t\tOUTER OBJECTS\n");
	std::cout << "\t\t(" << out_objects_list.size() << " elements)" << std::endl;

	if (out_objects_list.size() > 0)
		for (int out_idx = 0; out_idx < out_objects_list.size(); ++out_idx)
		{

			std::cout << "\n\t\tOBJECT " << out_idx << " --------------------------------------------------------------------------------------------------------------------" << std::endl;
			// Object type
			std::cout << "\t\tType -> " << in_objects_list[out_idx].GetObjectType() << std::endl << std::endl;
			// Reference pose
			std::cout << "\t\tReference pose -> out\n";
			// Actual pose
			matrix_to_print = in_objects_list[out_idx].GetObjectActualPose();
			std::cout << "\t\t\t\t\t|" << matrix_to_print(0,0) << "\t\t" << matrix_to_print(0,1) << "\t\t" << matrix_to_print(0,2) << "\t\t" << matrix_to_print(0,3) << "|\n"
					  << "\t\tActual pose ->"
					  << "\t\t|" << matrix_to_print(1,0) << "\t\t" << matrix_to_print(1,1) << "\t\t" << matrix_to_print(1,2) << "\t\t" << matrix_to_print(1,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(2,0) << "\t\t" << matrix_to_print(2,1) << "\t\t" << matrix_to_print(2,2) << "\t\t" << matrix_to_print(2,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(3,0) << "\t\t\t" << matrix_to_print(3,1) << "\t\t\t" << matrix_to_print(3,2) << "\t\t\t" << matrix_to_print(3,3) << "|\n\n";	
			// Transformation matrix
			matrix_to_print = in_objects_list[out_idx].GetObjectPoseTransformation();
			std::cout << "\t\t\t\t\t|" << matrix_to_print(0,0) << "\t\t" << matrix_to_print(0,1) << "\t\t" << matrix_to_print(0,2) << "\t\t" << matrix_to_print(0,3) << "|\n"
					  << "\t\tPose transformation ->"
					  << "\t|" << matrix_to_print(1,0) << "\t\t"<< matrix_to_print(1,1) << "\t\t" << matrix_to_print(1,2) << "\t\t" << matrix_to_print(1,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(2,0) << "\t\t" << matrix_to_print(2,1) << "\t\t" << matrix_to_print(2,2) << "\t\t" << matrix_to_print(2,3) << "|\n"
					  << "\t\t\t\t\t|" << matrix_to_print(3,0) << "\t\t\t" << matrix_to_print(3,1) << "\t\t\t" << matrix_to_print(3,2) << "\t\t\t" << matrix_to_print(3,3) << "|\n\n";	
		}
}

// VISUALIZING THE REFERENCE SCENE *********************************************************************************************************************************
void
SceneManagement::VisualizeRefScene()
{
	// Reference scene cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	// Visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));         
    // Viewport id
    int viewport = 1;
    // Viewports size
    double viewport_step = 1/(double)ref_objects_list.size();

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
	
    for (double i = 0; i < ref_objects_list.size(); ++i)
    {
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

		// Setting other visualizer settings and adding the cluster in his viewport
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color (ref_objects_list[i].GetObjectCloud(), color_vector[1], color_vector[2], color_vector[3]);

		scene_viewer->createViewPort(i*viewport_step,0,(i+1)*viewport_step,viewport_step,viewport);
		scene_viewer->addPointCloud<pcl::PointXYZ> (ref_objects_list[i].GetObjectCloud(), cluster_color, object_name.str().append("_info"), viewport);
        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, object_name.str().append("_info"));
		if (((int)i)%2 == 0)
			scene_viewer->setBackgroundColor(0.1,0.1,0.1,viewport+1);
		else
			scene_viewer->setBackgroundColor(0.15,0.15,0.15,viewport+1);
	    scene_viewer->resetCameraViewpoint(object_name.str().append("_info"));

	    Eigen::Matrix<float, 4, 4> object_pose;
	    object_pose = ref_objects_list[i].GetObjectRefPose();
	    std::string object_pose_string;
	    for(size_t matrix_i = 0; matrix_i < 4; ++matrix_i)
	    	for(size_t matrix_j = 0; matrix_j < 4; ++matrix_j)
	    	{
	    		std::stringstream ss;
  				ss << std::setprecision(std::numeric_limits<float>::digits10-3);
  				ss << object_pose(matrix_i,matrix_j);
	    		object_pose_string.append(ss.str());
	
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
	    scene_viewer->addPointCloud<pcl::PointXYZ> (ref_objects_list[i].GetObjectCloud(),cluster_color,object_name.str().append("_main"), 1);
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

// VISUALIZING REFERENCE SCENE AND ACTUAL SCENE *********************************************************************************************************************
void
SceneManagement::VisualizeActualScene()
{
	// Reference scene cloud pointer
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	// Visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));         

    // Visualizaton object initialization
    scene_viewer->initCameraParameters ();
    scene_viewer->addCoordinateSystem (1.0);
    scene_viewer->setBackgroundColor (0, 0, 0);
    scene_viewer->setWindowBorders(true);

    // Adding the whole Kinect acquisition (white)
    ref_cloud_ptr = this->GetActualScene();
	scene_viewer->addPointCloud<pcl::PointXYZ> (ref_cloud_ptr, "Whole_acquisition");
    scene_viewer->resetCameraViewpoint("Whole_acquisition");
    scene_viewer->addText("INNER AND OUTER OBJECTS IN THE ACTUAL CLOUD", 10, 10, 1, 1, 1, "AcquiredCloudText");
    scene_viewer->addText("_____ inner objects", 500, 10, 0, 1, 0, "InnerText");
    scene_viewer->addText("_____ outer objects", 500, 20, 1, 0, 0, "OuterText");
	scene_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText");

	// Adding inner objects
	for (int inner_idx = 0; inner_idx < in_objects_list.size(); ++inner_idx)
	{
		// Object identifier in the visualizer
    	std::stringstream object_name;
    	object_name << "object_in_" << inner_idx+1;
    	// Setting other visualizer settings and adding the cluster in his viewport
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color (in_objects_list[inner_idx].GetObjectCloud(), 0, 255, 0);
		// Adding the cloud
		scene_viewer->addPointCloud<pcl::PointXYZ> (in_objects_list[inner_idx].GetObjectCloud(), cluster_color, object_name.str().append("_info"));
        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, object_name.str().append("_info"));
   	    scene_viewer->resetCameraViewpoint(object_name.str().append("_info"));
	}

	// Adding outer objects
	for (int outer_idx = 0; outer_idx < out_objects_list.size(); ++outer_idx)
	{
		// Object identifier in the visualizer
    	std::stringstream object_name;
    	object_name << "object_out_" << outer_idx+1;
		// Setting other visualizer settings and adding the cluster in his viewport
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color (out_objects_list[outer_idx].GetObjectCloud(), 255, 0, 0);
		// Adding the cloud
		scene_viewer->addPointCloud<pcl::PointXYZ> (out_objects_list[outer_idx].GetObjectCloud(), cluster_color, object_name.str().append("_info"));
        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, object_name.str().append("_info"));
	    scene_viewer->resetCameraViewpoint(object_name.str().append("_info"));
	}

	// Viewer loop
    while (!scene_viewer->wasStopped ())
    { 
        scene_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    scene_viewer->close();
}