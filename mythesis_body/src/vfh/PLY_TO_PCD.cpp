// #include <ros/ros.h>
// #include <boost/thread/mutex.hpp>
// #include <image_transport/image_transport.h>
// //************************************************ PCL includes

// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>

// #include <ros/package.h>

// using namespace pcl;

// // void PointCloud_CB(const sensor_msgs::PointCloud2ConstPtr &cloud_m);

// int main(int argc, char **argv) {

// ros::init(argc, argv, "body_recognition");
// ros::NodeHandle n_[2];
 
 
// // ros::Subscriber sub1 = n_[0].subscribe("/camera/depth_registered/points", 1, PointCloud_CB);

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//   std::string path_read = ros::package::getPath("mythesis_body")+"/dataset/RGBD-ID/data/collaborative/01/ply/01.ply";
//   pcl::PLYReader reader;
//   reader.read<pcl::PointXYZ> (path_read, *cloud);

//   std::cerr << "Read cloud: " << std::endl;
//   std::cerr << *cloud << std::endl;

//   std::string path_write = ros::package::getPath("mythesis_body")+"/dataset/RGBD-ID/data/collaborative/01/ply/01.pcd";

//   pcl::PCDWriter pcdwriter;
//   pcdwriter.write<pcl::PointXYZ> (path_write, *cloud, false);

// // ros::Subscriber sub3 = n_[1].subscribe("/camera/rgb/image_color", 1, rosImageCallBack);
// //VTK_SHORT_MAX
// ros::spin();
// return 0;
// }
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
int
main (int argc, char** argv)
{
//  bool binary = true;

//  if(argc < 3) {
//    std::cerr << "Usage:" << std::endl;
//    std::cerr << argv[0] << " [-a] input.ply output.pcd" << std::endl;
//    std::cerr << "\t-a\tASCII output" << std::endl;
//    return (1);
//  }

//  if(argc == 4) {
//    if(strncmp(argv[1],"-a",2) != 0) {
//      std::cerr << "Error: unknown option!" << std::endl;
//      return (1);
//    }
//    else {
//      binary = false;
//      argv += 1;
//    }
//  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::io::loadPLYFile("/home/shaghayegh/catkin_ws/src/w.ply",*cloud);
//    pcl::io::loadPCDFile("/home/shaghayegh/catkin_ws/src/src14.pcd",*cloud);

      pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB> ("/home/shaghayegh/catkin_ws/08.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

       //blocks until the cloud is actually rendered
       viewer.showCloud(cloud);
    cv::waitKey(100);
       while (!viewer.wasStopped ())
          {
          //you can also do cool processing here
          //FIXME: Note that this is running in a separate thread from viewerPsycho
          //and you should guard against race conditions yourself...

          }
  // std::cerr << "Read cloud: " << std::endl;
  // std::cerr << *cloud << std::endl;

  // pcl::PCDWriter pcdwriter;
  // pcdwriter.write<pcl::PointXYZ> (argv[2], *cloud, binary);

  return (0);
}
