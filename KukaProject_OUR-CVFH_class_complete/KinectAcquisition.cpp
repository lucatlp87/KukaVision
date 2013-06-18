// <KinectAcquisition> CLASS METHODS IMPLEMENTATION

#include <iostream>

#include <pcl/console/print.h>

#include "Cloud.h"
#include "KinectAcquisition.h"
#include "KinectAcquisitionViewer.h"

// CONSTRUCTOR
KinectAcquisition::KinectAcquisition() {}

// DESTRUCTOR
KinectAcquisition::~KinectAcquisition() {}

// #################################################################################################################################################################
// ACQUIRING CLOUD FROM KINECT #####################################################################################################################################
// #################################################################################################################################################################

void 
KinectAcquisition::AcquireCloudKinect(Cloud cloud_container)

{
	// The function instantiate a <KinectAcquisitionViewer> class that starts the OpenNI grabber and a <pcl_visualizer> object. When the visualizer is closed, the 
	// actual cloud is stored in <point_cloud>. 

	// Acquisition viewer class instantiation
  	KinectAcquisitionViewer acquire;

    std::cout << "\tAcquiring the cloud ..." << std::flush;
  
  	// Acquiring function running
  	acquire.run ();
  	// Store the acquired cloud in the member <point_cloud> of the class
  	cloud_container.SetCloud(acquire.acquired_cloud);

  	std::cout << "\t(" << acquire.acquired_cloud.points.size() << " points)" << std::endl;
}