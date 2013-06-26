// <Cloud> CLASS METHODS IMPLEMENTATION

#include <iostream>

#include <pcl/console/print.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Cloud.h"

// CONSTRUCTOR
Cloud::Cloud() 
{
	seg_error = 0;
	no_clusters = 0;
}

// DESTRUCTOR
Cloud::~Cloud() {}

// #################################################################################################################################################################
// CLASS PRIVATE MEMEBERS I/O OPERATIONS ###########################################################################################################################
// #################################################################################################################################################################

// GETTING <seg_error> FLAG VALUE
bool 
Cloud::GetSegError()
{
	return seg_error;
}

// GETTING <no_clusters> FLAG VALUE
bool 
Cloud::GetNoClusters()
{
	return no_clusters;
}

// GETTING THE POINT CLOUD *****************************************************************************************************************************************
pcl::PointCloud<pcl::PointXYZ>::Ptr 
Cloud::GetCloud()
{
	// The function return a pointer to the <point_cloud> variable. If no point cloud is stored in <point_cloud> a warning is printed out. 
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_pointer = point_cloud;

	if (cloud_pointer == NULL)
		pcl::console::print_error ("	[WARNING] The cloud is empty!\n\n");
				
	return (cloud_pointer);
}

// GETTING THE NORMALS CLOUD ***************************************************************************************************************************************
pcl::PointCloud<pcl::Normal>::Ptr 
Cloud::GetNormals()
{
	// The function return a pointer to the <point_cloud_normals> variable. If no point cloud is stored in <point_cloud_normals> a warning is printed out. 

	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_pointer (new pcl::PointCloud<pcl::Normal>);
	*normals_cloud_pointer = point_cloud_normals;

	if (normals_cloud_pointer == NULL)
		pcl::console::print_error ("	[WARNING] The normals cloud is empty!\n\n");
				
	return (normals_cloud_pointer);
}

// GETTING THE OUR-CVFH HISTOGRAM **********************************************************************************************************************************
pcl::PointCloud<pcl::VFHSignature308>::Ptr 
Cloud::GetOURCVFH()
{
	// The function return a pointer to the <point_cloud_OURCVFH> variable. If no point cloud is stored in <point_cloud_OURCVFH> a warning is printed out. 

	pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH_pointer (new pcl::PointCloud<pcl::VFHSignature308>);
	*VFH_pointer = point_cloud_OURCVFH;

	if (VFH_pointer == NULL)
		pcl::console::print_error("		[WARNING] The OUR-CVFH signature is empty!\n\n");

	return (VFH_pointer);
}

// GETTING OUR-CVFH VECTOR *****************************************************************************************************************************************
std::vector<float>
Cloud::GetOURCVFHVector()
{
	// The function returns a vector of lfoats containing the OUR-CVFH signature
	std::vector<float> out_vector;
	out_vector.resize(308);

	for(size_t i = 0; i < 308; ++i)
		out_vector[i] = point_cloud_OURCVFH_vector[i];

	return(out_vector);
}

// SETTING THE CLOUD ***********************************************************************************************************************************************
void 
Cloud::SetCloud(pcl::PointCloud<pcl::PointXYZ> cloud_to_set)
{
	// The function sets the <point_cloud> variable with the input one

	point_cloud = cloud_to_set;
	
	return;
}

// SETTING THE NORMALS CLOUD ***************************************************************************************************************************************
void 
Cloud::SetNormals(pcl::PointCloud<pcl::Normal> normals_to_set)
{
	// The function sets the <point_cloud_normals> variable with the input one

	point_cloud_normals = normals_to_set;
	return;
}

// SET THE POINT CLOUD WITH NORMALS ********************************************************************************************************************************
void 
Cloud::SetCloudWithNormals(pcl::PointCloud<pcl::PointNormal> cloud_to_set)
{
	// The function sets the <point_cloud_with_normals> concatenating variables <point_cloud> and <point_cloud_normals>

	// pcl::concatenateFields(point_cloud,point_cloud_normals,point_cloud_with_normals);
	// point_cloud_with_normals.points.resize(cloud_to_set.points.size());
	point_cloud_with_normals = cloud_to_set;
}

// SETTING THE OUR-CVFH SIGNATURE **********************************************************************************************************************************
void 
Cloud::SetOURCVFH(pcl::PointCloud<pcl::VFHSignature308> histogram_to_set)
{
	// The function sets the <point_cloud_OURCVFH> variable with the input one

	point_cloud_OURCVFH = histogram_to_set;
	return;
}

// SETTING THE OUR-CVFH VECTOR *************************************************************************************************************************************
void
Cloud::SetOURCVFHVector()
{
	// The function set the <point_cloud_OURCVFH_vector> attribute

	pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
	signature_ptr = GetOURCVFH();
	// Point cloud fields object
	std::vector <sensor_msgs::PointField> fields;
	
	// OUR-CVFH vector
	std::vector<float> OURCVFH_vector;
	OURCVFH_vector.resize(308);

	// Getting the histogram values from the cloud
	pcl::getFieldIndex (*signature_ptr, "vfh", fields);

	// std::cout << point_cloud_OURCVFH.points[0] << std::endl;
	// Filling the vector
	for (size_t i = 0; i < 308; ++i)
		OURCVFH_vector[i] = signature_ptr->points[0].histogram[i];

	point_cloud_OURCVFH_vector = OURCVFH_vector;
}

// SETTING <seg_error> FLAG VALUE **********************************************************************************************************************************
void 
Cloud::SetSegError(bool flag_value)
{
		seg_error = flag_value;

}

// SETTING <no_clusters> FLAG VALUE ********************************************************************************************************************************
void 
Cloud::SetNoClusters(bool flag_value)
{
	no_clusters = flag_value;
}

// LOADING A CLOUD FROM A SPECIFIED PATH ***************************************************************************************************************************
void 
Cloud::LoadCloud(std::string path)
{
	// The function calls the PCL built-in function to load a point cloud from the specified path (<path>) and save it in the variable <point_cloud>
	pcl::io::loadPCDFile(path,point_cloud);
}

// LOADING A OUR-CVFH SIGNATURE FROM A SPECIFIED PATH **************************************************************************************************************
void
Cloud::LoadOURCVFH(std::string path)
{
	// The function calls the PCL built-in function to load a point cloud from the specified path (<path>) and save it in the variable <point_cloud_OURCVFH>

	pcl::io::loadPCDFile(path,point_cloud_OURCVFH);
}

// SAVING CLOUD DN OUR-CVFH SIGNATURE TO A SPECIFIED PATH **********************************************************************************************************
std::string 
Cloud::SaveCloud()
{
	// The function saves the <point_cloud_with_normals> memeber and the <point_cloud_OURCVFH> member. The path is inserted by user interactively. The functions is
	// designed in order to save clouds in the DB (the path to the DB is static and cannot be changed by the user) so the user is asked to insert both the name of 
	// the folder (<subfolder_name>) of the DB in which clouds have to be stored and the file name (<cloud_name>). Automatically the function saves the point cloud 
	// with the specified file name while add to it the sting "_ourcvfh" when saving the OUR-CVFH signature. All files are saved with ".pcd" extension.
	// The function returns the path corresponding to the updated DB folder.

	std::string cloud_dir ("../ObjectDB/"); 
	std::string subfolder_name;
	std::string cloud_name;
	std::stringstream path;
	std::stringstream path_ourcvfh;

	// DB subfolder interactive choice
	std::cout << "\t\tPlease insert the name of the subfolder in the DB: ";
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
	if (point_cloud_with_normals.points.size() == 0)
		pcl::console::print_error ("\n\t\t[ERROR] The point cloud with normals object has no points! No file .pcd will be saved!\n");
	else
	{
		std::cout << std::endl << "\t\tSaving the cloud with normals 	" << std::endl;
   		path << cloud_dir << cloud_name << ".pcd";
   		pcl::io::savePCDFile(path.str(),point_cloud_with_normals,false);
   	}

	// Saving OUR-CVFH signature
	if (point_cloud_OURCVFH.points.size() == 0)
		pcl::console::print_error ("\n\t\t[ERROR] The OUR-CVFH signature object has no points! No file .pcd will be saved!\n");
	else
	{
		std::cout << "\t\tSaving OUR-CVFH descriptors cloud 	" << std::endl;
    	path_ourcvfh << cloud_dir << cloud_name << "_ourcvfh.pcd";
    	pcl::io::savePCDFile(path_ourcvfh.str(),point_cloud_OURCVFH,false);
    }

	return cloud_dir;
}