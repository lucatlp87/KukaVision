// <TrainingStage> CLASS METHODS DEFINITION

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/StdVector>

#include <flann/flann.hpp>

#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "TrainingStageMesh.h"
#include "CloudProcessing.h"
#include "OURCVFHEstimation.h"
#include "DBManagement.h"

// CONSTRUCTOR
TrainingStageMesh::TrainingStageMesh() {}

// DESTRUCTOR
TrainingStageMesh::~TrainingStageMesh() {}

// #################################################################################################################################################################
// TRAINING STAGE IMPLEMENTATION ###################################################################################################################################
// #################################################################################################################################################################

void
TrainingStageMesh::RunStage()
{
	pcl::console::print_error("\n\n#######################################################################################");
	pcl::console::print_error("#########################################################\n");
	pcl::console::print_error("############################################### TRAINING STAGE MESH - OBJECT MODELS DB CREATION ");
	pcl::console::print_error("################################################\n");
    pcl::console::print_error("#######################################################################################");
	pcl::console::print_error("#########################################################\n");
    std::cout << "In this stage all synthetic meshes in the ObjectMeshesDB folder will be converted, filtered and stored "
              << "in ObjectDB folder" << std::endl << std::endl;

    
    // STEP TM0. USER PARSING ******************************************************************************************************************************
    // *****************************************************************************************************************************************************
    // The user is aked to choose between two possible operations:
    //      - execute the virtual scan of .ply object models
    //      - determine OUR-CVFH signatures of scans (this step requires a previous virtual scan)
    std::string user_choice;


    std::cout << "Choose between the following options:" << std::endl
              << "\t1. Execute virtual scanning of .ply objects model" << std::endl
              << "\t2. Execute OUR-CVFH estimation (this step requires a previous virtual scan)" << std::endl
              << "\t3. Exit" << std::endl
              << "\n\nYour choice: " << std::flush;
    std::cin >> user_choice;

    while(true)
    {
        if(!user_choice.compare("1") || !user_choice.compare("2") || !user_choice.compare("3"))
            break;
        else
        {
            pcl::console::print_error ("\n\t[COMMAND ERROR] Please insert a number between 1 and 3!\n\n");
            std::cout << "Your choice -> " << std::flush;
            std::cin >> user_choice;
        }
    }

    if(!user_choice.compare("1"))
    {
        // STEP TM1. SYNTHETIC OBJECT MODEL LOADING ************************************************************************************************************
        // *****************************************************************************************************************************************************
        // In this case, it is necessary that the folder ObjectMeshesDB contains some file .ply. If no file were found, no virtual scanning will be executed.
        // Once the virtual scan of a particular file is ended, the file will be moved into the AlreadyScanned subfolder of ObjectMeshesDB so that a new run of 
        // this script will not process it again. If the user want to re-scan that file, he has manually move it from AlreadyScanned sub-folder to ObjectMeshesDB
        // and run again this script.

        pcl::console::print_error ("\nSTEP TM1. OBJECT MESHES VIRTUAL SCANNING ******************************");
        pcl::console::print_error ("*************************************************************************\n");
        
        // INITIALIZATION
        // Creating the path of meshes DB
        std::string meshes_db = "../ObjectMeshesDB/";
        // Creating the path toi AlreadyScanned sub-folder
        std::string alreadyscanned_path = "../ObjectMeshesDB/AlreadyScanned/";
        // Path to the actual file
        std::stringstream file_ply_path;
        // Name of the actual file
        std::stringstream file_ply_name;
        // Name of the actual object
        std::string object_ply_name;
        // Path of the scanned file
        std::stringstream scanned_file_path;
        // Virtual scanning command line
        std::stringstream vs_cmd_line_str;
        // Command 
        const char* vs_cmd_line;

<<<<<<< HEAD
        // Timer
        pcl::console::TicToc tt;
=======
        // One scan timer
        pcl::console::TicToc tt;
        // Total loop timer
        pcl::console::TicToc tt_tot;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

        // Flag 
        bool there_is_object = 0;

<<<<<<< HEAD

        tt.tic();

        // VIRTUAL SCAN OF THE ALL .ply FILE IN THE FOLDER
        pcl::console::print_error("~~~ STARTING A NEW VIRTUAL SCAN\n\n");
        //std::cout << "(" << object_ply_name.c_str() << ")" << std::endl;
        vs_cmd_line_str << "/home/kukavision/carlos-pcl-trunk/build/bin/pcl_global_classification -models_dir ../ObjectMeshesDB/ -training_dir .";
        // vs_cmd_line_str << "/usr/local/bin/pcl_virtual_scanner -object_coordinates 0 -single_view 0 -view_point 1,0,0 -organized 0 -hor_res 0.05 -vert_res 0.05 " << file_ply_path.str().c_str();
        vs_cmd_line = new char;
        vs_cmd_line = vs_cmd_line_str.str().c_str();
        std::system(vs_cmd_line);

        vs_cmd_line_str.str("");

        there_is_object = 1;
        pcl::console::print_error("\n~~~ SCAN ENDED ");
        std::cout << "\n<(execution time: " << tt.toc() << " ms)>" << std::endl;
=======
        tt_tot.tic();

        // Create the sub_folder AlreadyScanned if not present
        if (!boost::filesystem::exists (alreadyscanned_path) && !boost::filesystem::is_directory(alreadyscanned_path))
            boost::filesystem::create_directory(alreadyscanned_path); 

        // Search for .ply file
        for (boost::filesystem::directory_iterator it_sub (meshes_db); it_sub != boost::filesystem::directory_iterator (); ++it_sub)
        {
            if (boost::filesystem::is_regular_file (it_sub->status ()) && boost::filesystem::extension (it_sub->path ()) == ".ply")
            {
                tt.tic();

                file_ply_path << it_sub->path().c_str();
                file_ply_name << file_ply_path.str().substr(file_ply_path.str().find_last_of("/")+1);
                object_ply_name = file_ply_name.str();
                object_ply_name.resize(object_ply_name.length()-4);

                // VIRTUAL SCAN OF THE ACTUAL .ply FILE
                pcl::console::print_error("~~~ STARTING A NEW VIRTUAL SCAN ");
                std::cout << "(" << object_ply_name.c_str() << ")" << std::endl;
                vs_cmd_line_str << "/usr/local/bin/pcl_virtual_scanner -object_coordinates 0 -single_view 0 -view_point 1,0,0 -organized 0 -hor_res 0.01 -vert_res 0.01 " << file_ply_path.str().c_str();
                vs_cmd_line = new char;
                vs_cmd_line = vs_cmd_line_str.str().c_str();
                std::system(vs_cmd_line);

                // Moving the scanned file in AlreadyScanned subfolder
                scanned_file_path << alreadyscanned_path.c_str() << file_ply_name.str().c_str();
                rename(file_ply_path.str().c_str(),scanned_file_path.str().c_str());


                file_ply_path.str("");
                file_ply_name.str("");
                object_ply_name = "";   
                scanned_file_path.str("");
                vs_cmd_line_str.str("");

                there_is_object = 1;
                pcl::console::print_error("\n~~~ SCAN ENDED ");
                std::cout << "\n<(execution time: " << tt.toc() << " ms)>" << std::endl;
            }
        }
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

        if (!there_is_object)
            pcl::console::print_error("\n~~~ [WARNING] No .ply file were found in ObjectMeshesDB directory!\n\n");
    }
    else if (!user_choice.compare("2"))
    {
        // OUR-CVFH SIGNATURE DETERMIANTION AND KD-TREE BUILD *************************************************************************************************
        // ****************************************************************************************************************************************************
        // In this case it is supposed that a virtual scanner was done previously.
        // The algorithm searches the for virtual scanner output folder in build directory. If there are no directory, the code will not mdify any file.
<<<<<<< HEAD
        // Each output folders contain 80 .pcd file: for each file the OUR-CVFH signature is calculated. At the end of this operation, the ouput folder in build
=======
        // Each output folders contain 42 .pcd file: for each file the OUR-CVFH signature is calculated. At the end of this operation, the ouput folder in build
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
        // directory will be deleted.
        // When all output folders are processed, the KD-Tree build and update routine is performed.

        // Warning the user that a virtual scanning has already been done
        pcl::console::print_error("\n~~~ STEP TM1. (VIRTUAL SCANNING) ALREADY DONE\n");

        // INITIALIZATION
        // Path to the current directory
        std::string build_path = "./";
        // Path to ObjectDB folder
        std::string objectdb_path = "../ObjectDB/";
        // Path to the output folder
        std::stringstream out_dir_path;
        // Name of the output folder
        std::stringstream out_dir_name;
        // Substring 'output'
        std::string is_output;
        // Name of the object
        std::stringstream obj_name;
        // Path to folder in ObjectDB
        std::stringstream new_folder_path;
        // Path of the view
        std::stringstream view_file_path;
        // Name of the view
        std::stringstream view_file_name;
        // Name of the actual .pcd file without extension
        std::string view_file_name_str;
        // Name of the view in ObjectDB
        std::stringstream final_view_file_name;
        // Path of the view in ObjectDB
        std::stringstream final_view_file_path;
        // Print on screen string
        std::stringstream print_view;
        // Path of the transformation matrix
        std::stringstream matrix_file_path;
        // Path of the signature in ObjectDB
        std::stringstream signature_file_path;
        // Path of the transformation matrix in ObjectDB
        std::stringstream final_matrix_file_path;
        // Remove the output folder command line
        std::stringstream rm_cmd_line_str;
        // Command line
        const char* rm_cmd_line;

        // First storage flag
        bool first_storage = 1;
        // Vector containing updated folders in ObjectDB
        std::vector<std::string> updated_folders (1,"NO_VIEW_SAVED");
<<<<<<< HEAD
        // Index of the view (1:80)
=======
        // Index of the view (1:42)
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
        int view_idx;
        // Flag one item found
        bool found = 0;

        // Current cluster pointer
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster;
        // Current normals pointer
        pcl::PointCloud<pcl::Normal>::Ptr current_normals;
        // Current cluster with normals pointer
        pcl::PointCloud<pcl::PointNormal>::Ptr current_cluster_normals;
        // Current OUR-CVFH signature pointer
        pcl::PointCloud<pcl::VFHSignature308>::Ptr current_signature;
        // Current pose matrix
        std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > current_transformation;

        // Processing class instantiation
        CloudProcessing processing_object;

        // Time counter
        pcl::console::TicToc tt;

        // ObjectDBB folder creation (if it does not exists)
        if (!boost::filesystem::exists (objectdb_path) && !boost::filesystem::is_directory(objectdb_path))
                        boost::filesystem::create_directory(objectdb_path);  

        // Search for virtual scanner output folder in build folder (current directory)
        for (boost::filesystem::directory_iterator it (build_path); it != boost::filesystem::directory_iterator (); ++it)
        {
            if (boost::filesystem::is_directory (it->path()))
            {
                // Get the actual folder path
                out_dir_path << it->path().c_str();
                // Extract the folder name
                out_dir_name << out_dir_path.str().substr(out_dir_path.str().find_last_of("/")+1);
                // Extract the 'output' piece
<<<<<<< HEAD
                // is_output = out_dir_name.str().substr(out_dir_name.str().find_last_of("_")+1);


                //Check if the actual folder is an output folder
                if (out_dir_name.str().compare("CMakeFiles") != 0)
=======
                is_output = out_dir_name.str().substr(out_dir_name.str().find_last_of("_")+1);


                //Check if the actual folder is an output folder
                if (!is_output.compare("output"))
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                {
                    found = 1;

                    // Extract the name of the object
<<<<<<< HEAD
                    obj_name << out_dir_name.str();
=======
                    obj_name << out_dir_name.str().substr(0,out_dir_name.str().find_last_of("."));
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                    pcl::console::print_error("\n\n~~~ PROCESSING NEW OBJECT ");
                    std::cout << "(" << obj_name.str() << ")" << std::endl;

                    // Create a folder with the same name of the object in ObjectDB folder
                    std::cout << "\tCreating folder in ObjectDB ..." << std::flush;
                    new_folder_path << objectdb_path.c_str() << obj_name.str().c_str() << "/";
                    if (!boost::filesystem::exists (new_folder_path.str()) && !boost::filesystem::is_directory(new_folder_path.str()))
                        boost::filesystem::create_directory(new_folder_path.str());  
                    if (first_storage)
                    {
                        updated_folders.assign(1, new_folder_path.str());
                        first_storage = 0;
                    }
                    else
                    {
                        bool insert_ok = 0;
                        for (int ret_idx = 0; ret_idx < updated_folders.size(); ++ret_idx)
                            if (updated_folders[ret_idx].compare(new_folder_path.str()) == 0)
                                insert_ok = 1;
                            if (insert_ok == 0)
                                updated_folders.push_back(new_folder_path.str());
                    }
                    std::cout << "\tdone" << std::endl;
                    
                    // Initialize the index of views
                    view_idx = 1;

                    // If the actual folder is an output folder it is necessary to explore its content
                    for (boost::filesystem::directory_iterator it_sub (out_dir_path.str()); it_sub != boost::filesystem::directory_iterator (); ++it_sub)
                    {
                        if (boost::filesystem::is_regular_file (it_sub->status ()) && boost::filesystem::extension (it_sub->path ()) == ".pcd")
                        {
                            // RESETTING POINTERS
                            // Resetting the cluster pointer;
                            current_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
                            // Resetting the current normals pointer
                            current_normals.reset(new pcl::PointCloud<pcl::Normal>);
                            // Resetting the current cluster with normals pointer
                            current_cluster_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
                            // Resetting the current signature pointer
                            current_signature.reset(new pcl::PointCloud<pcl::VFHSignature308>);

                            // Pose estimation utils
                            std::ifstream matrix_file_ptr;
                            std::ofstream matrix_final_file_ptr;
                            float matrix_element;
<<<<<<< HEAD
=======
                            // Pose of the camera in local object coordinates
                            Eigen::Matrix4f T_oc;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            // Pose of the object in camera coordinates
                            Eigen::Matrix4f T_co;
                            // Pose of the object wrt the signature frame in camera coordinates
                            Eigen::Matrix4f T_fo;

                            // Path of the .pcd of the actual view
                            view_file_path << it_sub->path().c_str();
                            // NAme of the actual .pcd file
                            view_file_name << view_file_path.str().substr(view_file_path.str().find_last_of("/")+1);
                            // Name of the actual .pcd file without extension
                            view_file_name_str = view_file_name.str();
                            view_file_name_str.resize(view_file_name_str.length()-4);
                            // Name of the .pcd file in ObjectDB
                            final_view_file_name << obj_name.str().c_str() << "_" << view_file_name.str().c_str(); 
                            // Path of the .pcd file in ObjectDB
                            final_view_file_path << objectdb_path << obj_name.str().c_str() << "/" << final_view_file_name.str().c_str();
                           
                            // Print info on screen
<<<<<<< HEAD
                            print_view << "\n~~~~~~~ PROCESSING " << view_file_name_str.c_str() << " (" << view_idx << " of 80)";
=======
                            print_view << "\n~~~~~~~ PROCESSING VIEW NUMBER " << view_file_name_str.c_str() << " (" << view_idx << " of 42)";
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            pcl::console::print_error(print_view.str().c_str());

                            // Loading the current view (.pcd file)
                            pcl::io::loadPCDFile(view_file_path.str(),*current_cluster);
                            int original_points = current_cluster->points.size();

                            // Getting the pose of the object in camera coordinates
<<<<<<< HEAD
                            matrix_file_path << view_file_path.str().substr(0,view_file_path.str().find_last_of("/")+1) << "pose" << view_file_name_str.substr(view_file_name_str.find_last_of("_")) << ".txt";
                            std::cout << "\n\tT_co loading..." << std::flush;
=======
                            matrix_file_path << view_file_path.str().substr(0,view_file_path.str().find_last_of(".")) << ".txt";
                            std::cout << "\n\tT_co determination..." << std::flush;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            matrix_file_ptr.open (matrix_file_path.str().c_str());    
                            for (int matrix_row = 0; matrix_row < 4; ++matrix_row)
                                for (int matrix_col = 0; matrix_col < 4; ++matrix_col)
                                {
<<<<<<< HEAD
                                    matrix_file_ptr >> matrix_element;
                                    T_co(matrix_row,matrix_col) = matrix_element;
                                }
                            matrix_file_ptr.close();
=======
                                        matrix_file_ptr >> matrix_element;
                                    T_oc(matrix_row,matrix_col) = matrix_element;
                                }
                            matrix_file_ptr.close();
                            T_co = T_oc.inverse();
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            std::cout << "\tdone" << std::endl;

                            // STEP TM1.1. MLS FILTER APPLICATION AND NORMAL ESTIMATION *******************************************************************
                            // ****************************************************************************************************************************
<<<<<<< HEAD
                            pcl::console::print_error("\n\tSTEP TM2.1. MLS FILTER APPLICATION AND NORMALS ESTIMATION ********");
                            pcl::console::print_error ("**********************************************************************");

                            // boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer_1 (new pcl::visualization::PCLVisualizer ("Scene Viewer"));     
                            // scene_viewer_1->initCameraParameters ();
                            // scene_viewer_1->addCoordinateSystem (1.0);
                            // scene_viewer_1->setBackgroundColor (0, 0, 0);
                            // scene_viewer_1->setWindowBorders(true);
                            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_midori_1 (current_cluster, 227, 249, 136);

                            // scene_viewer_1->addPointCloud<pcl::PointXYZ> (current_cluster, color_midori_1, "Scene");
                            // scene_viewer_1->resetCameraViewpoint("Scene");
                            // scene_viewer_1->addText("CHOSEN REFERENCE SCENE", 10, 10, 1, 0, 0, "SceneText");
                            // scene_viewer_1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Scene");
                            // scene_viewer_1->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "SceneText");

                            // while (!scene_viewer_1->wasStopped ())
                            // { 
                            //     scene_viewer_1->spinOnce (100);
                            //     boost::this_thread::sleep (boost::posix_time::microseconds (1000));
                            // }
                            // scene_viewer_1->close();

=======
                            pcl::console::print_error("\n\tSTEP TM2.1. MLS FILTER APPLICATION AND NORMALS ESTIMATION *********");
                            pcl::console::print_error ("*********************************************************************");
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                            tt.tic();
                            std::cout << "\n\tMLS filter application and normals computation..." << std::flush;

<<<<<<< HEAD
                            processing_object.MLSFilterAndNormalsComputation(current_cluster,current_normals,current_cluster_normals,0);
=======
                            processing_object.MLSFilterAndNormalsComputation(current_cluster,current_normals,current_cluster_normals);
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                            std::cout << "\tdone" << std::endl;
                            std::cout << "\t(from " << original_points << " to " << current_cluster->points.size() << " data points)" << std::endl
                                      << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

<<<<<<< HEAD
                            // boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer_2 (new pcl::visualization::PCLVisualizer ("Scene Viewer"));     
                            // scene_viewer_2->initCameraParameters ();
                            // scene_viewer_2->addCoordinateSystem (1.0);
                            // scene_viewer_2->setBackgroundColor (0, 0, 0);
                            // scene_viewer_2->setWindowBorders(true);
                            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_midori_2 (current_cluster, 227, 249, 136);

                            // scene_viewer_2->addPointCloud<pcl::PointXYZ> (current_cluster, color_midori_2, "Scene");
                            // scene_viewer_2->resetCameraViewpoint("Scene");
                            // scene_viewer_2->addText("CHOSEN REFERENCE SCENE", 10, 10, 1, 0, 0, "SceneText");
                            // scene_viewer_2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Scene");
                            // scene_viewer_2->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, "SceneText");

                            // while (!scene_viewer_2->wasStopped ())
                            // { 
                            //     scene_viewer_2->spinOnce (100);
                            //     boost::this_thread::sleep (boost::posix_time::microseconds (1000));
                            // }
                            // scene_viewer_2->close();




=======
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            // STEP TM1.2. OUR-CVFH SIGNATURE COMPUTATION *********************************************************************************
                            // ****************************************************************************************************************************
                            pcl::console::print_error("\n\tSTEP TM2.2. OUR-CVFH SIGNATURE COMPUTATION ************************");
                            pcl::console::print_error ("*********************************************************************");

                            tt.tic();
                            std::cout << "\n\tOUR-CVFH signature estimation..." << std::flush;
                                
                            // OUR-CVFH estimation class 
                            OURCVFHEstimation ourcvfh_object;
                            ourcvfh_object.CloudOURCVFHComputation(current_cluster,current_normals,current_signature, current_transformation);
          
                            std::cout << "\tdone" << std::endl;
                            std::cout << "\t" << current_signature->points.size() << " smooth region(s) found" << std::endl;
                            std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;

                            // STEP TM1.3. SIGNATURE STORAGE *********************************************************************************************
                            // ****************************************************************************************************************************
                            pcl::console::print_error("\n\tSTEP TM2.3. SIGNATURE STORAGE ************************************");
<<<<<<< HEAD
                            pcl::console::print_error ("**********************************************************************");
=======
                            pcl::console::print_error ("*********************************************************************");
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                            tt.tic();
                            // SAVING CLOUD WITH NORMALS --------------------------------------------------------------------------------------------------
                            std::cout << "\n\tSaving cluster with normals ..." << std::flush;
                            pcl::io::savePCDFile(final_view_file_path.str(),*current_cluster_normals,false);
                            std::cout << "\tdone" << std::endl;
<<<<<<< HEAD

=======
                   
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                            // SAVING OUR-CVFH SIGNATURES -----------------------------------------------------------------------------------------------
                            std::cout << "\tSaving OUR-CVFH signatures ..." << std::endl;
                            if (current_signature->points.size() == 0)
                                pcl::console::print_error ("\n\t\t[ERROR] The OUR-CVFH signature object has no points! No file .pcd will be saved!\n");
                            else
                            {
                                pcl::PointCloud<pcl::VFHSignature308>::Ptr actual_surface (new pcl::PointCloud<pcl::VFHSignature308>);
                                actual_surface->width = 1; actual_surface->height = 1;
                                actual_surface->points.resize(1);
                                    
                                for (size_t signature_idx = 0; signature_idx < current_signature->points.size(); ++signature_idx)
                                {
                                    signature_file_path << objectdb_path.c_str() << obj_name.str().c_str() << "/" << obj_name.str().c_str() << "_" << view_file_name_str.c_str() << "_" << signature_idx+1 << "_ourcvfh.pcd";
                                  
                                    actual_surface->points[0] = current_signature->points[0,signature_idx];        
                                    pcl::io::savePCDFile(signature_file_path.str(),*actual_surface,false);

                                    // Pose of the object wrt the signature frame in camera coordinates determination
                                    std::cout << "\t\tT_fo (signature " << signature_idx+1 << ") determination and storage..." << std::flush;
<<<<<<< HEAD
                                    T_fo = current_transformation[signature_idx]*T_co;   
=======
                                    T_fo = current_transformation[signature_idx].inverse()*T_co;    
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
                                    // Matrix storage       
                                    final_matrix_file_path << signature_file_path.str().substr(0,signature_file_path.str().find_last_of(".")) << ".txt";                     
                                    matrix_final_file_ptr.open(final_matrix_file_path.str().c_str());
                                    matrix_final_file_ptr << T_fo;
                                    matrix_final_file_ptr.close();
                                    std::cout << "\tdone" << std::endl;

                                    signature_file_path.str("");
                                    final_matrix_file_path.str("");
                                }
                            }
                            std::cout << "\t<(execution time: " << tt.toc() << " ms)>" << std::endl;
                            
                            // Resetting string & stringstream
                            view_file_path.str("");
                            view_file_name.str("");
                            view_file_name_str = "";
                            final_view_file_name.str("");
                            final_view_file_path.str("");
                            print_view.str("");
                            matrix_file_path.str("");
                            ++view_idx;
                        }
                    }
                    // Delete the processed output folder
                    rm_cmd_line_str << "rm -r "  << out_dir_path.str().c_str();
                    rm_cmd_line = new char;
                    rm_cmd_line = rm_cmd_line_str.str().c_str();
                    std::system(rm_cmd_line);
                    rm_cmd_line_str.str("");
                }
                // Reset all strings
                obj_name.str("");
                out_dir_path.str("");
                out_dir_name.str("");
                is_output = "";
                new_folder_path.str("");
            }
        }
        
        if (!found)
        {
            pcl::console::print_error("\n~~~ STEP TM2. OUR-CVFH SIGNATURE ESTIMATION\n");
            pcl::console::print_error("\n\t[WARNING] NO OUTPUT FOLDER FOUND!\n\n");
        }


        // STEP TM3. KDTREE UPDATE *********************************************************************************************************************************
        // ********************************************************************************************************************************************************
        tt.tic();
        pcl::console::print_error ("\n~~~ STEP TM3. KD-TREES UPDATE *************************************************************************************************\n");
              
        // Prcessing the modified subfolder vector. 
        if (!updated_folders[0].compare("NO_VIEW_SAVED"))
            // If the vector <updated_folders> is empty (the first element is equal to NO_CLUSTERS_SAVED) this step is skipped and a warining message is printed out.
            pcl::console::print_error("\n\t[WARNING] No new clusters added! KdTrees are up to date!\n");
        else
        {
            // Time counter
            pcl::console::TicToc tt_update;

            std::cout << "(" << updated_folders.size() << " modified subfolder(s))" << std::endl;

            // The update procedure is performed for each modified DB subfolder.
            for (int loop_idx = 0; loop_idx < updated_folders.size(); ++loop_idx)
            {
                // Update class instantiation
                DBManagement update_object;
                  
                std::cout << std::endl << "UPDATING THE FOLDER " << updated_folders[loop_idx] << std::flush;

                // Update all paths and search the actual subfolder for .pcd files.
                update_object.UpdateObject(updated_folders[loop_idx]);
                std::cout << "(" << update_object.GetFoundSignatureNumber() << " histograms found)" << std::endl;
                  
                // STEP T4.1 OUR-CVFH HISTOGRAM CONVERSION TO FLANN FORMAT *********************************************************************************************
                // *****************************************************************************************************************************************************
                // The first step of the update procedure deals with the conversion of OUR-CVFH histogram point cloud into a Flann matrix. Once the matrix is created 
                // and filled it is saved in the current subfolder of the DB.
                tt_update.tic();
<<<<<<< HEAD
                pcl::console::print_error ("\tSTEP TM3.1. OUR-CVFH histrogram conversion to Flann format...");
=======
                pcl::console::print_error ("\tSTEP TM2.1. OUR-CVFH histrogram conversion to Flann format...");
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                // Flann conversion
                update_object.OURCVFHFlannConversion(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                // STEP T4.2 OUR-CVFH HISTOGRAM FILE NAME STORAGE ******************************************************************************************************
                // *****************************************************************************************************************************************************
                // The second step consist in adding (in append mode) the new histrogrms paths to the file containing names of hall istograms of the current subfolder.

                tt_update.tic();
<<<<<<< HEAD
                pcl::console::print_error ("\n\tSTEP TM3.2. OUR-CVFH histograms path add...");
=======
                pcl::console::print_error ("\n\tSTEP TM2.2. OUR-CVFH histograms path add...");
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                // Name list update
                update_object.HistogramNameStorage(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;

                // STEP T4.3 CREATION, BUILD AND STORAGE OF THE KD-TREE ***********************************************************************************************
                // ****************************************************************************************************************************************************
                // The last step consists in creating and build a tree index about all elements of the current subfolder.

                tt_update.tic();
<<<<<<< HEAD
                pcl::console::print_error ("\n\tSTEP TM3.3. Kd-Tree index build and storage...");
=======
                pcl::console::print_error ("\n\tSTEP TM2.3. Kd-Tree index build and storage...");
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

                // Index building and storage
                update_object.BuildOURCVFHTreeIndex(updated_folders[loop_idx]);

                pcl::console::print_error ("\tdone\n");
                std::cout << "\t<(execution time: " << tt_update.toc() << " ms)>" << std::endl;
            }
        }
        std::cout << std::endl << "---> KD-TREES UPDATE total execution time: " << tt.toc() << " ms" << std::endl << std::endl;
    }
    return;
}
    