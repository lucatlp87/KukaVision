// CLASS <KinectAcquisition> DEFINITION
// The class <KinectAcquisition> contains the method that implements the OpenNI grabber in order to acquire a point cloud by Kinect sensor.

class KinectAcquisition
{
public:
	// *************************************************************************************
	// Constructor
	KinectAcquisition();
	// Destructor
	~KinectAcquisition();

	// ACQUIRING CLOUD BY KINECT SENSOR ****************************************************
	// Acquire cloud from Kinect sensor
	void AcquireCloudKinect(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
};