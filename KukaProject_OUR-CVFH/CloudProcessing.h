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
	// Point data density uniformation
	void VoxelGridFilter(Cloud *input_cloud);

	// CLOUD PROCESSING *********************************************************************************************************************************************
	// 	Clusters extraction
	std::vector<Cloud, Eigen::aligned_allocator<Cloud> > ExtractClustersFromCloud (Cloud input_cloud);
	// Normals computation
	void CloudNormalsComputation (Cloud *input_cloud);
};