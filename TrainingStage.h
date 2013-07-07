// CLASS <TrainingStage> DEFINITION	
// In the class <TrainingStage> the procedure of the object models DB creation is implemented.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TrainingStage
{
public:
	// **************************************************************************************************************************************************************
	// Constructor
	TrainingStage();
	// Destructor
	~TrainingStage();

	// TRAINING STAGE IMPLEMENTATION ********************************************************************************************************************************
	void RunStage();
};