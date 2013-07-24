// CLASS <KukaVisionStage> DEFINITION	
// In the class <KukaVisionStage> the procedure of the object models DB creation is implemented.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class KukaVisionStage
{
public:
	// **************************************************************************************************************************************************************
	// Constructor
	KukaVisionStage();
	// Destructor
	~KukaVisionStage();

	// TRAINING STAGE IMPLEMENTATION ********************************************************************************************************************************
	void RunStage();
};