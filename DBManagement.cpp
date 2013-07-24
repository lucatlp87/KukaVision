// <DBManagement> CLASS METHODS IMPLEMENTATION

#include <fstream>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "DBManagement.h"

// CONSTRUCTOR
DBManagement::DBManagement() 
{
	default_flann_file = "FlannMatrixOURCVFH.h5";
	default_name_file = "ModelNames.list";
	default_tree_file = "KdTree.index";
}

// DESTRUCTOR
DBManagement::~DBManagement() {}

// #################################################################################################################################################################
// I/O OPERATIONS ON PRIVATE MEMBERS ###############################################################################################################################
// #################################################################################################################################################################

// GETTING THE NUMBER OF FOUND SIGNATURE
int 
DBManagement::GetFoundSignatureNumber()
{
	if (sub_elements.size() == 0)
		pcl::console::print_error ("\n\t[WARNING] No signatures found!\n\n");
	else
		return(sub_elements.size());
}

// #################################################################################################################################################################
// CLASS OBJECT UPDATE #############################################################################################################################################
// #################################################################################################################################################################

// SEARCHING FOR .pcd FILES IN THE ACTUAL SUBFOLDER ****************************************************************************************************************
void 
DBManagement::SearchTheSubfolder()
{
	// Search the current DB subfolder for .pcd files
	for (boost::filesystem::directory_iterator it (sub_path); it != boost::filesystem::directory_iterator (); ++it)
	{
		if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == ".pcd")
	   	{
	   		// When a .pcd file is found it is necessary to check fields of the cloud in order to select only cloud corresponding to OUR-CVFH histograms
	   		
	   		// Temporary model pair
	   		model_pair actual_file;
			int vfh_idx;

	   		// Load the file as a PCD
	   		try
	   		{
			    sensor_msgs::PointCloud2 cloud;
			    int version;
			    Eigen::Vector4f origin;
			    Eigen::Quaternionf orientation;
			    pcl::PCDReader r;
			    int type; unsigned int idx;
			      
			    r.readHeader (it->path().string(), cloud, origin, orientation, version, type, idx);
			    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
			      			
			    // The following condition allows to consider only files containig VFH signatures
			    if (vfh_idx == 0 && (int)cloud.width * cloud.height == 1)
			    {
			  		// Treat the VFH signature as a single Point Cloud
			    	pcl::PointCloud <pcl::VFHSignature308> point;
			    	pcl::io::loadPCDFile (it->path().string (), point);
			    	actual_file.second.resize (308);

		    		std::vector <sensor_msgs::PointField> fields;
		    		pcl::getFieldIndex (point, "vfh", fields);

			       	// Add the histogram to the model pair
		      		for (size_t i = 0; i < fields[vfh_idx].count; ++i)
		    	   		actual_file.second[i] = point.points[0].histogram[i];
		    	
		    	    // Add the file path to the model pair
		    		actual_file.first = it->path().string ();

		    		// Insert the actual model pair in the vector vfh_list
		  			sub_elements.push_back(actual_file);
			    }
			}
			catch (pcl::InvalidConversionException e)
			{
				break;							
			}
	   	}
	}
}

// UPDATING CLASS OBJECT *******************************************************************************************************************************************
void 
DBManagement::UpdateObject(std::string base_path)
{
	actual_flann_path = base_path;
	actual_flann_path.append(default_flann_file);
    actual_name_path = base_path;
	actual_name_path.append(default_name_file);
	actual_tree_path = base_path;
	actual_tree_path.append(default_tree_file);

	sub_path = base_path;

	this->SearchTheSubfolder();
}

// #################################################################################################################################################################
// DB UPDATE OPERATIONS ############################################################################################################################################
// #################################################################################################################################################################

// CONVERTING HISTOGRAM POINT CLOUD IN FLANN FORMAT ****************************************************************************************************************
void 
DBManagement::OURCVFHFlannConversion(std::string input_folder)
{
	// Flann matrix instantiation
	flann::Matrix<float> ourcvfh_signatures_flann (new float[sub_elements.size () * sub_elements[0].second.size ()], 
													  sub_elements.size (), sub_elements[0].second.size ());
		    	
	// Matrix fill
	for (size_t i = 0; i < ourcvfh_signatures_flann.rows; ++i)
	  	for (size_t j = 0; j < ourcvfh_signatures_flann.cols; ++j)
	  		ourcvfh_signatures_flann[i][j] = sub_elements[i].second[j];

	// Matrix storage
	flann::save_to_file (ourcvfh_signatures_flann, actual_flann_path, "training_data");
	flann_signatures = ourcvfh_signatures_flann;
}

// SAVING HISTOGRAM FILE NAME LIST *********************************************************************************************************************************
void 
DBManagement::HistogramNameStorage(std::string input_folder)
{
    // Pointer of the file containing alla models name in the actual subfolder
  	std::ofstream name_file_ptr;
    name_file_ptr.open (actual_name_path.c_str());

    // File update (new paths are added in append mode)
    for (size_t i = 0; i < sub_elements.size (); ++i)
     	name_file_ptr << sub_elements[i].first << "\n";
    
    // Close the file
    name_file_ptr.close ();
}
	
// BUILDING THE TREE INDEX AND SAVING IT
void 
DBManagement::BuildOURCVFHTreeIndex(std::string input_folder)
{
	// The first step consists in checking Flann matrix dimension
	if (flann_signatures.rows == 0 || flann_signatures.cols == 0)
		pcl::console::print_error ("\n\t[WARNING] The matrix that is being used to build the tree is empty!");
	else
	{
		// Index instantiation
		flann::Index<flann::ChiSquareDistance<float> > index (flann_signatures, flann::KDTreeIndexParams (4));
	   	// Index build
	   	index.buildIndex ();
	   	// Index storage
		index.save (actual_tree_path);
	}
}

// #################################################################################################################################################################
// OUR-CVFH CORRESPONDENCES SEARCH #################################################################################################################################
// #################################################################################################################################################################

// SEARCHING OUR-CVFH CORRESPONDENCES

bool
DBManagement::SearchTheDB(std::vector<float> input_signature)
{
	// UPDATING THE SEARCH FOLDER



	// SECURITY CHECK **********************************************************************************************************************************************
	// In this section we check if the current subfolder contains files <FlannMatrixOURCVFH.h5>, <ModelName.list> and <KdTree.index>

	// Searching for files
	if (!boost::filesystem::exists(actual_flann_path) || !boost::filesystem::exists(actual_name_path) || !boost::filesystem::exists(actual_tree_path))
	{
		pcl::console::print_error("\n\t[FATAL ERROR] In folder %s one of the following file is missing:\n", sub_path.c_str());
		pcl::console::print_error("\t\tFlannMatrixVFH.h5\n");
		pcl::console::print_error("\t\tModelNames.list\n");
		pcl::console::print_error("\t\tKdTree.index\n");
		return (0);
	}

	// INITIALIZATION **********************************************************************************************************************************************
	// Return variable
	bool found = 0;
	// Number of acceptable correspondences
	int k = 1;
	// Vector containing OUR-CVFH model pairs
	std::vector<model_pair> model_vector;
	// Flann matrix containing
  	flann::Matrix<int> k_indices (new int[k], 1, k);
  	// Flann matrix containing distances between model and correspondences
  	flann::Matrix<float> k_distances (new float[k], 1, k);
  	// Flann matrix containing OUR-CVFH histograms data
  	flann::Matrix<float> histogram_data;
	
  	// Loading histograms data (Flann format) in the Flann matrix
    flann::load_from_file (histogram_data, actual_flann_path, "training_data");
	
	// Loading file names in the <path> variable of VFH pair vector
	std::ifstream name_file_ptr;
	std::string line;
	name_file_ptr.open (actual_name_path.c_str());

	while (!name_file_ptr.eof ())
	{
	  getline (name_file_ptr, line);
	  if (line.empty ())
	    continue;
	  model_pair m;
	  m.first = line;
	  model_vector.push_back (m);
	}
	name_file_ptr.close ();

	// Building the tree
	flann::Index<flann::ChiSquareDistance<float> > index (histogram_data, flann::SavedIndexParams (actual_tree_path));
    index.buildIndex ();


    // SEARCHING THE TREE ******************************************************************************************************************************************
    // The last section deals with the search algorithm 

  	flann::Matrix<float> p = flann::Matrix<float>(new float[308], 1, 308);
  
  	memcpy (&p.ptr ()[0], &input_signature[0], p.cols * p.rows * sizeof (float));

  	index.knnSearch (p, k_indices, k_distances, k, flann::SearchParams (512));
  	delete[] p.ptr ();

	if (k_distances[0][0] > 5)
  		std::cout << "\tNo correspondences found" << std::endl;
  	else
  	{
  		// Output the results on screen
		pcl::console::print_error("\n\t\t\tCorrespondence found:\n");
		std::cout << "\t\t\t\tfile -> " << model_vector.at (k_indices[0][0]).first.c_str () << std::endl
				  << "\t\t\t\twith distance equal to " << k_distances[0][0] << std::endl;
		found = 1;
	}
	return (found);
}