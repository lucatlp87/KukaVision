// CLASS <DBManagement> DEFINITION	
// In this file the <DBManagement> class id defined. It caontains methods used to perfrom Kd-Tree search in the DataBase

#include <flann/flann.h>
typedef std::pair<bool, float> match_pair;

class DBManagement
{
private:
	typedef std::pair<std::string, std::vector<float> > model_pair;

	// Default name of the file contianing the Flann matrix
	std::string default_flann_file;
	// Default name of the file containing the list of paths corresponding to the subfolder elements
	std::string default_name_file;
	// Default name of the file conataining the tree
	std::string default_tree_file;

	// Actual subfolder path in boost format
	boost::filesystem::path sub_path;

	// Actual path to the file containing the Flann matrix
	std::string actual_flann_path;
	// Actual path to the file containing the name list
	std::string actual_name_path;
	// Actual path to the file containing the tree
	std::string actual_tree_path;

	// Vector containing all model_pair of the actual subfolder
	std::vector<model_pair> sub_elements;

	// Histogram flann conversion matrix container
	flann::Matrix<float> flann_signatures; 
public:
	// **************************************************************************************************************************************************************
	// Constructor
	DBManagement();
	// Destructor
	~DBManagement();

	// I/O OPERATIONS ON PRIVATE MEMEBERS ***************************************************************************************************************************
	// Get the number of found signatures
	int GetFoundSignatureNumber();

	// CLASS OBJECT UPDATE ******************************************************************************************************************************************
	// Search the number of signatures in the actual subfolder
	void SearchTheSubfolder();
	// Update paths and fill the list of found signature
	void UpdateObject(std::string base_path);

	// DB UPDATE OPERATIONS *****************************************************************************************************************************************
	// Point cloud histograms conversion to Flann format
	void OURCVFHFlannConversion(std::string input_folder);
	// Histogram name list storage
	void HistogramNameStorage(std::string input_folder);
	// Build the tree index and save it
	void BuildOURCVFHTreeIndex(std::string input_folder);

	// OUR-CVFH CORRESPONDENCES SEARCH ******************************************************************************************************************************
	// Search the current sub-folder for correspondences
	match_pair SearchTheDB(std::vector<float> input_signature, int points_number);
};
