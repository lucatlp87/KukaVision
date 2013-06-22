// CLASS <OURCVFHEstimation> DEFINITION
// The class <OURCVFHEstimation> contains all methods used in OUR-CVFH signature estimation.

class OURCVFHEstimation
{
public:
	// **************************************************************************************************************************************************************
	// Constructor	
	OURCVFHEstimation();
	// Destructor
	~OURCVFHEstimation();

	// OUR-CVFH SIGNATURES ESTIMATION *******************************************************************************************************************************
	// Compute OUR-VFH signatures
	void CloudOURCVFHComputation(Cloud *input_cloud);
};