#include <iostream>

#include <pcl/console/print.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>

// CLASS DEFINITION
// The following class aims to instantiate and manage a visualizer for the scene seen by Kinect sensor.
class KinectAcquisitionViewer
{
  public:
    KinectAcquisitionViewer () : viewer ("Kinect sensor viewer") {}

    void 
    cloud_visualization (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &visualized_cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (visualized_cloud);
      else
        acquired_cloud = *visualized_cloud;
    }

    void 
    run ()
    {
      pcl::Grabber *interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&KinectAcquisitionViewer::cloud_visualization, this, _1);

      interface->registerCallback (f);
      interface->start ();

      pcl::console::print_error("\tPlease, quit the viewer to acquire the cloud\n");

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
    pcl::PointCloud<pcl::PointXYZ> acquired_cloud;
};
