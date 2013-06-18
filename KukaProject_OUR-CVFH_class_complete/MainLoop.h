// CLASS <MainLoop> DEFINITION	
// In this file the <MainLoop> class id defined. It contains methods corresponding to tha available stages of the whole proceduere as described in the initial 
// section of file <main.cpp>

class MainLoop
{
public:
	// **************************************************************************************************************************************************************
	// Constructor
	MainLoop();
	// Desctructor
	~MainLoop();

	// HELP *********************************************************************************************************************************************************
	void PrintUsage(const char* progName);

	// TRAINING STAGE ***********************************************************************************************************************************************
	void TrainingStage();

	// NEW REFERENCE SCENE CREATION *********************************************************************************************************************************
	void NewSceneStage();

	// KUKA VISION IMPLEMENTATION ***********************************************************************************************************************************
	void KukaVisionStage();

};