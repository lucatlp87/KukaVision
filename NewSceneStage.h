// CLASS <NewSceneStage> DEFINITION	
// In the class <NewSceneStage> the procedure of the object models DB creation is implemented.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class NewSceneStage
{
public:
	// **************************************************************************************************************************************************************
	// Constructor
	NewSceneStage();
	// Destructor
	~NewSceneStage();

	// TRAINING STAGE IMPLEMENTATION ********************************************************************************************************************************
	void RunStage();
};