// <MaintenanceStage> CLASS METHODS DEFINITION

#include <fstream>

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "MaintenanceStage.h"
#include "KinectAcquisition.h"
#include "CloudProcessing.h"

// CONSTRUCTOR
MaintenanceStage::MaintenanceStage() {}

// DESTRUCTOR
MaintenanceStage::~MaintenanceStage() {}

// #################################################################################################################################################################
// MAINTENANCE STAGE IMPLEMENTATION ################################################################################################################################
// #################################################################################################################################################################

void
MaintenanceStage::RunStage()
{
	pcl::console::print_error("\n\n#######################################################################################");
	pcl::console::print_error("#######################################################################################\n");
	pcl::console::print_error("################################################################# MAINTENANCE STAGE - DBs AND CAMERA SET UP");
	pcl::console::print_error(" #################################################################\n");
    pcl::console::print_error("#######################################################################################");
	pcl::console::print_error("#######################################################################################\n");
    
	// INITIALIZATION
	// User choice
	std::string user_choice;
	// Object to delete
	std::string obj_to_delete;
	// ObjectDB base path
	std::string obj_base = "../ObjectDB/";
	// Object to delete path
	boost::filesystem::path obj_path;
	// Scene to delete
	std::string scene_to_delete;
	// SceneDB base path
	std::string scene_base = "../SceneDB/";
	// Scene to delete path
	boost::filesystem::path scene_path;

	bool found = 0;


	// User command parsing
    std::cout << std::endl << "Please select what to delete and repeat the operation as many times you want" << std::endl
    		  << "\t 1. Delete an object model (this will delete the whole subfolder in ObjectDB" << std::endl
    		  << "\t 2. Delete a reference scene (this will delete the whole subfolder in SceneDB" << std::endl
    		  << "\t 3. Set up the camera" << std::endl
    		  << "\t 4. Exit" << std::endl << std::endl
    		  << "Insert the number corresponding to your choice -> " << std::flush;
   	std::cin >> user_choice;

   	while(true)
   	{
   		if(!user_choice.compare("1") || !user_choice.compare("2") || !user_choice.compare("3") || !user_choice.compare("4"))
   			break;
   		else
   		{
   			pcl::console::print_error ("\n\t\t[COMMAND ERROR] Please insert <y> or <n>\n\n");
   			std::cout << "Insert the number corresponding to your choice -> " << std::flush;
   			std::cin >> user_choice;
   		}
   	}

   	if (!user_choice.compare("1"))
   	{
   		pcl::console::print_error("\nDELETING AN OBJECT MODEL ***********************************************************************");
   		pcl::console::print_error("*******************************************************************************\n\n");
   		
   		// Path to the DB directory
		boost::filesystem::path ObjectDB_path = "../ObjectDB";
		// Iterator
		int folder_number = 0;

		for (boost::filesystem::directory_iterator it (ObjectDB_path); it != boost::filesystem::directory_iterator (); ++it)
		    if (boost::filesystem::is_directory (it->status ()))
		    {
		        std::stringstream folder_name;
		        folder_name << it->path ().c_str();
		        std::cout << "\t" << folder_name.str().substr(folder_name.str().find_last_of("/")+1) << std::endl;
		        ++folder_number;
		    }

		if (folder_number == 0)
		{
		    std::cout << "\tNo models are stored in the DB!\n\n" << std::endl;
		    return;
		}
		else
		{
			std::cout << "\nPlease enter the name of the folder you want to delete: " << std::flush;
			std::cin >> obj_to_delete;
			obj_base.append(obj_to_delete);
			obj_path = obj_base;

			for (boost::filesystem::directory_iterator it (ObjectDB_path); it != boost::filesystem::directory_iterator (); ++it)
				if (boost::filesystem::is_directory (it->status ()))
	   				if (it->path() == obj_path)
	   				{
	   					found = 1;
	   					boost::filesystem::remove_all(obj_path);
	   				}

			if(!found)
				pcl::console::print_error ("\n\tThe folder you specified does not exist!\n\n");
		}
   	}
   	else if (!user_choice.compare("2"))
   	{
   		pcl::console::print_error("\nDELETING A REFERENCE SCENE ***********************************************************************");
   		pcl::console::print_error("*******************************************************************************\n\n");
   		
   		// Path to the DB directory
		boost::filesystem::path SceneDB_path = "../SceneDB";
		// Iterator
		int folder_number = 0;

		for (boost::filesystem::directory_iterator it (SceneDB_path); it != boost::filesystem::directory_iterator (); ++it)
		    if (boost::filesystem::is_directory (it->status ()))
		    {
		        std::stringstream folder_name;
		        folder_name << it->path ().c_str();
		        std::cout << "\t" << folder_name.str().substr(folder_name.str().find_last_of("/")+1) << std::endl;
		        ++folder_number;
		    }

		if (folder_number == 0)
		{
		    std::cout << "\tNo models are stored in the DB!\n\n" << std::endl;
		    return;
		}
		else
		{
			std::cout << "\nPlease enter the name of the folder you want to delete: " << std::flush;
			std::cin >> scene_to_delete;
			scene_base.append(scene_to_delete);
			scene_path = scene_base;

			for (boost::filesystem::directory_iterator it (SceneDB_path); it != boost::filesystem::directory_iterator (); ++it)
				if (boost::filesystem::is_directory (it->status ()))
	   				if (it->path() == scene_path)
	   				{
	   					found = 1;
	   					boost::filesystem::remove_all(scene_path);
	   				}

			if(!found)
				pcl::console::print_error ("\n\tThe folder you specified does not exist!\n\n");
		}
   	}
   	else if (!user_choice.compare("3"))
   	{
   		pcl::console::print_error("\nSETTING UP THE CAMERA FOR THE SCENE ACQUISITION ******************************************");
   		pcl::console::print_error("*************************************************************************************\n\n");
   		std::cout << "\tPlease connect the Kinect device and start an acquisition. Set up the device to the ultimate position from which other acquisition will be done." << std::endl;

		// Path to the configuration file
		boost::filesystem::path config_file_path = "../ConfigParams.txt";
		// Pointer to the config file
  		std::ofstream config_file_ptr;
   		// User choice
   		std::string set_up_choice;
   		// KinectAcquisition class instantiation
   		KinectAcquisition setting_acquisition;
   		// CloudProcessing class instantiation
   		CloudProcessing cloud_processing;
   		// Acquired point cloud container
   		pcl::PointCloud<pcl::PointXYZ>::Ptr acquired_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   		// PassThrough filter bounds
   		float x_min, x_max, y_min, y_max, z_min, z_max;   

   		// ACQUIRING THE CLOUD
   		setting_acquisition.AcquireCloudKinect(acquired_cloud);

   		std::cout << "\n\tDo you want to set PassThrough filter parameters? " << std::flush;
   		std::cin >> set_up_choice;

   		while (true)
   		{
   			if(!set_up_choice.compare("y") || !set_up_choice.compare("n"))
   				break;
   			else
   			{
   				pcl::console::print_error("\n\t[WARNING] Invalid choice! You can insert ""y"" or ""n""\n");
   				std::cout << "\tPlease insert your choice " << std::flush;
   				std::cin >> set_up_choice;
   			}
   		}

   		if(!set_up_choice.compare("y"))
   		{
   			while (true)
   			{
   				// SETTING PARAMETERS
   				std::cout << "\n\tPlease insert the min bound for x axis: " << std::flush;
   				std::cin >> x_min;
   				std::cout << "\tPlease insert the max bound for x axis: " << std::flush;
   				std::cin >> x_max;
   				std::cout << "\tPlease insert the min bound for y axis: " << std::flush;
   				std::cin >> y_min;
   				std::cout << "\tPlease insert the max bound for y axis: " << std::flush;
   				std::cin >> y_max;
   				std::cout << "\tPlease insert the min bound for z axis: " << std::flush;
	   			std::cin >> z_min;
   				std::cout << "\tPlease insert the max bound for z axis: " << std::flush;
   				std::cin >> z_max;

   				// Writing parameters to configuration file
   				pcl::console::print_error ("\n\tUpdating the file...");

   				config_file_ptr.open(config_file_path.c_str());
   				config_file_ptr << x_min << "\n"
   								<< x_max << "\n"
   								<< y_min << "\n"
   								<< y_max << "\n"
   								<< z_min << "\n"
   								<< z_max << "\n";
	   			config_file_ptr.close();
	   			
	   			pcl::console::print_error("\tdone\n");

	   			// FILTER APPLICATION
	   			pcl::console::print_error("\n\tApplying the filter...");
	   			
	   			cloud_processing.PassThroughFilter(acquired_cloud);
	   			
	   			pcl::console::print_error("\tdone\n");

	   			// VISUALIZING FILTERED ACQUISITION
	   			pcl::console::print_error("\n\tVisualizing the filtered cloud...");
	   			pcl::console::print_error("\n\tPlease close the visualizer in order to continue...");
				
				// Visualization object
    			boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_scene_viewer (new pcl::visualization::PCLVisualizer ("Scene Viewer"));    
    			filtered_scene_viewer->initCameraParameters ();
    			filtered_scene_viewer->addCoordinateSystem (1.0);
    			filtered_scene_viewer->setBackgroundColor (0, 0, 0);
    			filtered_scene_viewer->setWindowBorders(true);

    			filtered_scene_viewer->addPointCloud<pcl::PointXYZ> (acquired_cloud, "Whole_acquisition");
    			filtered_scene_viewer->resetCameraViewpoint("Whole_acquisition");
    			filtered_scene_viewer->addText("PASS-THROUGH FILTER SETTING UP", 10, 10, 1, 1, 1, "AcquiredCloudText");
				filtered_scene_viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "AcquiredCloudText");

				// Viewer loop
    			while (!filtered_scene_viewer->wasStopped ())
			    { 
			        filtered_scene_viewer->spinOnce (100);
			        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
			    }
			    filtered_scene_viewer->close();

			    // Asking the user for a new setting
	   			std::string reset_choice;

	   			while(true)
	   			{

	   				std::cout << "\n\n\tDo you want to set again parameters? " << std::flush;
	   				std::cin >> reset_choice;

	   				if(!reset_choice.compare("y") || !reset_choice.compare("n"))
	   					break;
	   				else
	   					pcl::console::print_error("\n\t[WARNING] Please enter one among ""y"" and ""n""\n\n");
	   			}
	   			if(!reset_choice.compare("n"))
	   				break;
	   		}
   		}
   	}
   	
   	return;
}