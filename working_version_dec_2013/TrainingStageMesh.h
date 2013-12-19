// CLASS <TrainingStage> DEFINITION	
// In the class <TrainingStage> the procedure of the object models DB creation is implemented.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TrainingStageMesh
{
public:
	// **************************************************************************************************************************************************************
	// Constructor
	TrainingStageMesh();
	// Destructor
	~TrainingStageMesh();

	// TRAINING STAGE IMPLEMENTATION ********************************************************************************************************************************
	void RunStage();
};