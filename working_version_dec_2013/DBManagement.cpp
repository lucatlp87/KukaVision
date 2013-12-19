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
			    pcl::PCLPointCloud2 cloud;
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

		    		std::vector <pcl::PCLPointField> fields;
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
	// Deleting the previous flann matrix
	boost::filesystem::path old_matrix;
	old_matrix = actual_flann_path;
	boost::filesystem::remove(actual_flann_path);

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
		// Delete the previous index
		boost::filesystem::path old_index;
		old_index = actual_tree_path;
		boost::filesystem::remove(old_index);

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

match_pair
DBManagement::SearchTheDB(std::vector<float> input_signature, int points_number)
{
	// Output structure
	match_pair output_pair;

	// SECURITY CHECK **********************************************************************************************************************************************
	// In this section we check if the current subfolder contains files <FlannMatrixOURCVFH.h5>, <ModelName.list> and <KdTree.index>

	// Searching for files
	if (!boost::filesystem::exists(actual_flann_path) || !boost::filesystem::exists(actual_name_path) || !boost::filesystem::exists(actual_tree_path))
	{
		pcl::console::print_error("\n\t[FATAL ERROR] In folder %s one of the following file is missing:\n", sub_path.c_str());
		pcl::console::print_error("\t\tFlannMatrixVFH.h5\n");
		pcl::console::print_error("\t\tModelNames.list\n");
		pcl::console::print_error("\t\tKdTree.index\n");
		output_pair.first = "File missing";
<<<<<<< HEAD
		output_pair.second[0] = -1;
=======
		output_pair.second = -1;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
		return (output_pair);
	}

	// INITIALIZATION **********************************************************************************************************************************************
	// Number of acceptable correspondences
	int k = 10;
	// Sum of signature elements values
	float sign_sum = 0;
	// Output match vector
	std::vector<float> match_vec;
	// Mean of first 10 matches
	float mean_up = 0;
	// Mean of last 10 matches
	float mean_below = 0;
	// Identification ratio
<<<<<<< HEAD
	std::vector<float> id_ratio;
=======
	float id_ratio;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
	// Found/Not found flag
	bool found = 0;
	// Vector containing OUR-CVFH model pairs
	std::vector<model_pair> model_vector;
	// Flann matrix containing
  	flann::Matrix<int> k_indices (new int[k], 1, k);
  	// Flann matrix containing distances between model and correspondences
  	flann::Matrix<float> k_distances (new float[k], 1, k);
  	// Flann matrix containing OUR-CVFH histograms data
  	flann::Matrix<float> histogram_data;
  	// Eigen matrix in which the local transformation is stored
  	Eigen::Matrix4f local_transform;

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

  	// Calculate the distance between signatures
  	for (int ii = 0; ii < input_signature.size(); ++ii)
  		sign_sum = sign_sum + input_signature[ii];
  	for (int jj = 0; jj < k; ++jj)
  		id_ratio.push_back(1-k_distances[0][jj]/sign_sum);
 //  	// Caluclating the mean between the first k matches
 //  	int loop_idx = 1;
	// for (std::vector<float>::iterator m_idx = id_ratio.begin(); m_idx != id_ratio.end(); ++m_idx)
	// {
	// 	match_sum += (*m_idx)*100;
	// 	if (loop_idx >= k-10)
	// 		match_sum_below += (*m_idx)*100;
	// 	++loop_idx;
	// }
	// match_mean = match_sum/k;
	// match_mean_below = match_sum_below/10;
  	
  	// Extraction of the higest match 

  	// Determination of the mean of first 10 matches
  	for (int mean_idx_up = 0; mean_idx_up < 5; ++mean_idx_up)
  	{
  		mean_up = mean_up + id_ratio[mean_idx_up]*100;
  	}
  	mean_up = mean_up/5;
  	match_vec.push_back(mean_up);
  	// // Determination of the mean of last 10 matches
  	// for (int mean_idx = k-10; mean_idx < k; ++mean_idx)
  	// {
  	// 	mean_below = mean_below + id_ratio[mean_idx]*100;
  	// }
  	// mean_below = mean_below/10;
  	// match_vec.push_back(mean_below);

  	match_vec.push_back(0);
  	match_vec.push_back(0);
  	// int occ = 0;
  	// for (int i = 0; i < 80; ++i)
  	// 	if (id_ratio[i]*100 > 97)
  	// 		++occ;











	if (match_vec[0] < 70)
	{
  		std::cout << "\tNo correspondences found" << std::endl;
<<<<<<< HEAD
		std::cout << "\t\t\tMaximum fitting percentage (" << k << " matches): " << match_vec[0] << " %" << std::endl;
		output_pair.first = "No match";
		output_pair.second.push_back(0);
		output_pair.second.push_back(0);
=======
  		if (id_ratio > 1)
  			id_ratio = 2;
		std::cout << "\t\t\tMinimum found distance: " << k_distances[0][0] << " (" << (1-id_ratio)*100 << "%%" << " fitting)" << std::endl;
		output_pair.first = "No match";
		output_pair.second = 0;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
	}
  	else
  	{
  		// Output the results on screen
<<<<<<< HEAD
		pcl::console::print_error("\n\t\t\t\tCorrespondence found:");
		std::cout << "\n\t\t\t\t\tfile -> " << model_vector.at (k_indices[0][0]).first.c_str () << std::endl;
		std::cout << "\t\t\t\t\tBest " << k << " matches:" << std::endl;
		for (int dd = 0; dd < id_ratio.size(); ++dd)
			std::cout << "\t\t\t\t\tdistance equal to " << k_distances[0][dd] << "\t(" << (id_ratio[dd])*100 << " %" << " fitting)" << std::endl;
		std::cout << "\t\t\t\t\tMaximum fitting percentage (" << k << " matches):\t" << match_vec[0] << " %" << std::endl;
		std::cout << "\t\t\t\t\tAverage fitting percentage (first 10 matches):\t" << match_vec[1] << " %" << std::endl;
		std::cout << "\t\t\t\t\tAverage fitting percentage (last 10 matches):\t" << match_vec[2] << " %" << std::endl;
=======
		pcl::console::print_error("\n\t\t\t\tCorrespondence found:\n");
		std::cout << "\t\t\t\t\tfile -> " << model_vector.at (k_indices[0][0]).first.c_str () << std::endl
				  << "\t\t\t\t\twith distance equal to " << k_distances[0][0] << " (" << (1-id_ratio)*100 << "%%" << " fitting)" << std::endl;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf

		// Retrieving the local transformation
		std::stringstream local_transform_path;
		local_transform_path << model_vector.at(k_indices[0][0]).first.substr(0,model_vector.at(k_indices[0][0]).first.find_last_of(".")) << ".txt";
		output_pair.first = local_transform_path.str();
<<<<<<< HEAD
		output_pair.second = match_vec;
=======
		output_pair.second = (1-id_ratio)*100;
>>>>>>> c0078ba5d23d49eecb39a09b9d75f77e4a19ecbf
		local_transform_path.str("");
	}

	return (output_pair);
}