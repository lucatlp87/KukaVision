// CLASS <CloudProcessing> DEFINITION
// The class <CloudProcessing> contains all methods used in cloud filtering and clusters extraction.

class CloudProcessing
{
public:
	// **************************************************************************************************************************************************************
	// Constructor	
	CloudProcessing();
	// Destructor
	~CloudProcessing();


	// CLOUD FILTERS ************************************************************************************************************************************************
	// Pass-through filter
	void PassThroughFilter(Cloud input_cloud);
	// Voxel grid filter
	void MLSFilterAndNormalsComputation(Cloud *input_cloud);
	// 

	// CLOUD PROCESSING *********************************************************************************************************************************************
	// 	Clusters extraction
	std::vector<Cloud, Eigen::aligned_allocator<Cloud> > ExtractClustersFromCloud (Cloud input_cloud);
};