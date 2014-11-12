#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
// C++ includes.
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

// OpenCV includes.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
//from nearest neighbors
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
//from don segmentation
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>



#include <XnCppWrapper.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <upperbodycore_msgs/Skeleton.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
//********************************************** cv_bridge
#include <cv_bridge/cv_bridge.h>
//***************************************************Pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


using namespace cv;
using namespace std;
using namespace pcl;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
{
	Mat camImage;
	// Mat depthImage;
    if ( cloud_m->width != 0 )
    {
        pcl::fromROSMsg(*cloud_m, *global_cloud);
        sensor_msgs::Image image;
        pcl::toROSMsg (*cloud_m, image);
        cv_bridge::CvImagePtr cv_ptrImage,cv_ptrDepthImage;
            cv_ptrImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            if (cv_ptrImage->image.size().width > 0 )
            {
                cv_ptrImage->image.copyTo(camImage);
            }
            imshow("salam" ,camImage);
            waitKey(10);
     }
}
int main(int argc, char* argv[])
{
        ros::init(argc, argv, "pcd_to_image");
        ros::NodeHandle nh_[1];
        ros::Subscriber sub1 = nh_[0].subscribe("/camera/depth_registered/points", 1, Cloud_CallBack);
        ros::spin();
}

