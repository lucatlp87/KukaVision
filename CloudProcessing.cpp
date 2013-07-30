// <CloudProcessing> CLASS METHODS IMPLEMENTATION

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

#include "CloudProcessing.h"

// CONSTRUCTOR
CloudProcessing::CloudProcessing() {}

// DESTRUCTOR
CloudProcessing::~CloudProcessing() {}

// ##################################################################################################################################################################
// POINT CLOUD FILTERS ##############################################################################################################################################
// ##################################################################################################################################################################

// PASS-THROUGH FILTER **********************************************************************************************************************************************
void 
CloudProcessing::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	  // The function deals with the application of a Pass-through filter in order to extract from the acquired point cloud the region corresponding to the tabletop

	  // Pass-through filter object
	  pcl::PassThrough<pcl::PointXYZ> PTfilter;
    // Configuration file pointer
    std::ifstream config_file_ptr;
    // Parameters container
    std::vector<float> filter_params;
    filter_params.resize(6);
    
    // Loding filter parameters
    config_file_ptr.open ("../ConfigParams.txt");

    std::cout << "aleeeeeee" << std::endl;
    for (int file_idx = 0; file_idx < 6; ++file_idx)
    {
      std::string line;
      getline (config_file_ptr, line);
      filter_params[file_idx] = boost::lexical_cast<float>(line.c_str());
    }

    config_file_ptr.close();

	  // Filter object initialization
	  PTfilter.setInputCloud (input_cloud);
	  PTfilter.setFilterFieldName ("x");
  	PTfilter.setFilterLimits (filter_params[0], filter_params[1]);
  	PTfilter.setFilterFieldName ("y");
  	PTfilter.setFilterLimits (filter_params[2], filter_params[3]);
  	PTfilter.setFilterFieldName ("z");
  	PTfilter.setFilterLimits (filter_params[4], filter_params[5]);

  	// Filter application
  	PTfilter.filter (*input_cloud);
}

// VOXEL GRID FILTER ************************************************************************************************************************************************
void 
CloudProcessing::MLSFilterAndNormalsComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, pcl::PointCloud<pcl::Normal>::Ptr normals,
												                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	 // The function deals with the application of a Voxel Grid Filter with leaf size of 3 mm (the best resolution that Kinct sensor offers)

  	// Instantiation of the MLS object 
	  boost::shared_ptr<pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> > MLSFilter (new pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>);
	  // KD-Tree
  	boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZ> > tree (new pcl::search::KdTree<pcl::PointXYZ>);

	  // Filter object initialization
    MLSFilter->setInputCloud (cluster);
    MLSFilter->setComputeNormals(true);
    MLSFilter->setPolynomialFit(true);
    MLSFilter->setSearchMethod(tree);
    MLSFilter->setSearchRadius(0.006);
	  MLSFilter->setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
 	  MLSFilter->setDilationIterations (2);
	  MLSFilter->setDilationVoxelSize (0.004f);

	  // Filter application
  	MLSFilter->process(*cloud_with_normals);

  	// Storage of results in the <Cloud> class in input 
  	cluster->points.resize(cloud_with_normals->points.size());
  	normals->points.resize(cloud_with_normals->points.size());

	  for (size_t i = 0; i < cloud_with_normals->points.size(); i++) 
	  {
        // Copying PointXYZ data
    	  cloud_with_normals->points[i].x = cluster->points[i].x;
    	  cloud_with_normals->points[i].y = cluster->points[i].y;
    	  cloud_with_normals->points[i].z = cluster->points[i].z;
    	  // Copying of Normal data
    	  cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
    	  cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
    	  cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
    }
}

// ##################################################################################################################################################################
// POINT CLOUD STORAGE ##############################################################################################################################################
// ##################################################################################################################################################################

// LISTING DB ITEMS ************************************************************************************************************************************************
void
CloudProcessing::ListDBItems()
{
  // Path to the DB directory
  boost::filesystem::path ObjectDB_path = "../ObjectDB";
  // Iterator
  int folder_number = 0;

  for (boost::filesystem::directory_iterator it (ObjectDB_path); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
          std::stringstream folder_name;
            folder_name << it->path ().c_str();
            std::cout << "\t\t\t" << folder_name.str().substr(folder_name.str().find_last_of("/")+1) << std::endl;
            ++folder_number;
        }
    }

    if (folder_number == 0)
      std::cout << "\t\t\tNo models are stored in the DB!" << std::endl;
}

// SAVING THE CLOUD ************************************************************************************************************************************************
std::string
CloudProcessing::SaveCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cluster_with_normals, pcl::PointCloud<pcl::VFHSignature308>::Ptr OURCVFH_signature)
{
  // The function saves the <point_cloud_with_normals> memeber and the <point_cloud_OURCVFH> member. The path is inserted by user interactively. The functions is
  // designed in order to save clouds in the DB (the path to the DB is static and cannot be changed by the user) so the user is asked to insert both the name of 
  // the folder (<subfolder_name>) of the DB in which clouds have to be stored and the file name (<cloud_name>). Automatically the function saves the point cloud 
  // with the specified file name while add to it the sting "_ourcvfh" when saving the OUR-CVFH signature. All files are saved with ".pcd" extension.
  // The function returns the path corresponding to the updated DB folder.

  std::string cloud_dir ("../ObjectDB/"); 
  // If the DB does not exist it will be created
  if (!boost::filesystem::exists (cloud_dir) && !boost::filesystem::is_directory(cloud_dir))
    boost::filesystem::create_directory(cloud_dir);
  
  std::string subfolder_name;
  std::string cloud_name;
  std::stringstream path;
  std::stringstream path_ourcvfh;

  // Listing available models
  std::cout << "\t\tAvailable models are:" << std::endl << std::endl;
  this->ListDBItems();
  std::cout << std::endl << "\t\t(if this is a new model, please insert a new folder: it will be automatically created in the DB)" << std::endl;

  // DB subfolder interactive choice
  std::cout << "\n\t\tPlease insert the name of the subfolder in the DB: ";
  std::cin >> subfolder_name;
  subfolder_name.append("/");
  cloud_dir.append(subfolder_name);

  

  // If the specified directory does not exist in the DB it will be created
  if (!boost::filesystem::exists (cloud_dir) && !boost::filesystem::is_directory (cloud_dir))
    boost::filesystem::create_directory(cloud_dir);

  // file name interactive choice       
  std::cout << "\t\tPlease insert the name of the cloud: ";
  std::cin >> cloud_name;

  // Saving the cluster with normals
  if (cluster_with_normals->points.size() == 0)
    pcl::console::print_error ("\n\t\t[ERROR] The point cloud with normals object has no points! No file .pcd will be saved!\n");
  else
  {
    std::cout << std::endl << "\t\tSaving the cloud with normals  " << std::endl;
    path << cloud_dir << cloud_name << ".pcd";
    pcl::io::savePCDFile(path.str(),*cluster_with_normals,false);
  }

  // Saving OUR-CVFH signature
  if (OURCVFH_signature->points.size() == 0)
    pcl::console::print_error ("\n\t\t[ERROR] The OUR-CVFH signature object has no points! No file .pcd will be saved!\n");
  else
  {
    std::cout << "\t\tSaving OUR-CVFH descriptors cloud   " << std::endl;
    path_ourcvfh << cloud_dir << cloud_name << "_ourcvfh.pcd";

    pcl::io::savePCDFile(path_ourcvfh.str(),*OURCVFH_signature,false);
  }

  return cloud_dir;
}