// <CloudProcessing> CLASS METHODS IMPLEMENTATION

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <Eigen/StdVector>

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
												                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, int dilation_it)
{
	 // The function deals with the application of a Voxel Grid Filter with leaf size of 3 mm (the best resolution that Kinct sensor offers)

  	// Instantiation of the MLS object 
	  boost::shared_ptr<pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> > MLSFilter (new pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>);
	  // KD-Tree
  	boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZ> > tree (new pcl::search::KdTree<pcl::PointXYZ>);

	  // Filter object initialization
    if(dilation_it == 0)
    {
        MLSFilter->setInputCloud (cluster);
        MLSFilter->setComputeNormals(true);
        MLSFilter->setPolynomialFit(true);
        MLSFilter->setSearchMethod(tree);
        MLSFilter->setSearchRadius(0.003);
        MLSFilter->setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
        MLSFilter->setDilationIterations (3);
        MLSFilter->setDilationVoxelSize (0.003f);
    }
    else
    {
        // MLSFilter->setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
        // MLSFilter->setInputCloud (cluster);
        // MLSFilter->setComputeNormals(true);
        // MLSFilter->setPolynomialFit(true);
        // MLSFilter->setSearchMethod(tree);
        // MLSFilter->setSearchRadius(0.01);
        // MLSFilter->setPointDensity(500);
        
        MLSFilter->setInputCloud (cluster);
        MLSFilter->setComputeNormals(true);
        MLSFilter->setPolynomialFit(true);
        MLSFilter->setSearchMethod(tree);
        MLSFilter->setSearchRadius(0.003);
        MLSFilter->setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
        MLSFilter->setDilationIterations (1);
        MLSFilter->setDilationVoxelSize (0.003f);
    }
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
CloudProcessing::SaveCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cluster_with_normals, pcl::PointCloud<pcl::VFHSignature308>::Ptr OURCVFH_signature,
                           std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > signature_pose)
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
  std::stringstream path_matrix;

  float matrix_element;
  Eigen::Matrix4f T_co;
  Eigen::Matrix4f T_fo;
  std::ofstream matrix_final_file_ptr;

  // Listing available models
  std::cout << "\t\tAvailable models are:" << std::endl << std::endl;
  this->ListDBItems();
  std::cout << std::endl << "\t\t(if this is a new model, please insert a new folder: it will be automatically created in the DB)" << std::endl;

  // DB subfolder interactive choice
  std::cout << "\n\t\tPlease insert the name of the subfolder in the DB: ";
  std::cin >> subfolder_name;
  subfolder_name.append("/");
  cloud_dir.append(subfolder_name);

  // If the specified subfolder does not exist in the DB it will be created
  if (!boost::filesystem::exists (cloud_dir) && !boost::filesystem::is_directory (cloud_dir))
    boost::filesystem::create_directory(cloud_dir);

  // file name interactive choice       
  std::cout << "\t\tPlease insert the name of the cloud: ";
  std::cin >> cloud_name;

  // Asking for the actual fixed object frame
  std::cout << "\t\tPlease insert the actual orientation of the object fixed frame (by row):" << std::endl;
  for (size_t row_idx = 0; row_idx < 3; ++row_idx)
      for (size_t col_idx = 0; col_idx < 3; ++col_idx)
      {
          std::cout << "\t\t\tpos (" << row_idx+1 << "," << col_idx+1 << ") : " << std::flush;
          std::cin >> matrix_element;
          T_co(row_idx,col_idx) = matrix_element;
      }
  std::cout << "\t\tPlease insert the actual position of the origin of the object fixed frame:" << std::endl;
  for (size_t row_idx = 0; row_idx < 3; ++row_idx)
  {
      std::cout << "\t\t\tpos (" << row_idx+1 << ",4) : " << std::flush;
      std::cin >> matrix_element;
      T_co(row_idx,3) = matrix_element;
  }
  // Inserting the fourth row of the pose matrix
  T_co(3,0) = 0.00; T_co(3,1) = 0.00; T_co(3,2) = 0.00; T_co(3,3) = 1.00;

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

    pcl::PointCloud<pcl::VFHSignature308>::Ptr actual_surface (new pcl::PointCloud<pcl::VFHSignature308>);
    actual_surface->width = 1; actual_surface->height = 1;
    actual_surface->points.resize(1);
    
    for (size_t idx=0; idx<OURCVFH_signature->points.size(); ++idx)
    {
        // Saving the OUR-CVFH signature
        path_ourcvfh << cloud_dir << cloud_name << "_" << idx+1 << "_ourcvfh.pcd";
  
        actual_surface->points[0] = OURCVFH_signature->points[0,idx];        
        pcl::io::savePCDFile(path_ourcvfh.str(),*actual_surface,false);

        // Saving the transformation matrix between signature and fixed object frame
        path_matrix << cloud_dir << cloud_name << "_" << idx+1 << "_ourcvfh.txt";

        T_fo = signature_pose[idx].inverse()*T_co;
                std::cout << T_co << std::endl << std::endl
                  << signature_pose[idx] << std::endl << std::endl
                  << T_fo << std::endl << std::endl;


        matrix_final_file_ptr.open(path_matrix.str().c_str());
        matrix_final_file_ptr << T_fo;
        matrix_final_file_ptr.close();

        path_ourcvfh.str("");
        path_matrix.str("");
    }
  }
  return cloud_dir;
}