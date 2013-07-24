// <MaintenanceStage> CLASS METHODS DEFINITION

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/console/print.h>

#include "MaintenanceStage.h"

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
	pcl::console::print_error("################################################################# MAINTENANCE STAGE - OBJECT MODELS CREATION");
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
    		  << "\t 3. Exit" << std::endl << std::endl
    		  << "Insert the number corresponding to your choice -> " << std::flush;
   	std::cin >> user_choice;

   	while(true)
   	{
   		if(!user_choice.compare("1") || !user_choice.compare("2") || !user_choice.compare("3"))
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
   	
   	return;
}