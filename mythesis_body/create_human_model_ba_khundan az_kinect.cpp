#include <iostream>
#include <stdio.h>
#include "omp.h"
#include <boost/thread/thread.hpp>
#include <math.h>
#include <boost/filesystem.hpp>
// timer
#include <cstdio>
#include <ctime>
//*********************************************** ROS includes
#include <stdexcept>
// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <image_transport/image_transport.h>
//************************************************ PCL includes
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>//*****************this and befor nessecery for recognition.hpp
//************************************************keypoint
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>
//************************************************openCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include "opencv2/calib3d/calib3d.hpp"
//********************************************** cv_bridge
#include <cv_bridge/cv_bridge.h>
// messages
//#include <object_recognition/coordinate.h>
#include "upperbodycore_msgs/coordinate.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <ros/package.h>
//jost for this code

#include <iostream>
#include <tuple>
//#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include "OpenNI.h"
#include <stdio.h>
#include <iostream>
#include "omp.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <pcl/features/normal_3d_omp.h>
using namespace pcl;
using namespace std;
using namespace cv;
 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
// boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer ("3D Viewer"));
class NormalEstimator{
public:
  pcl::NormalEstimationOMP<PointXYZRGB, pcl::PointNormal> norm_est;

  NormalEstimator() {
    norm_est.setKSearch (10);
  }

  NormalEstimator(int n_neighbours){//: NormalEstimator() {//ameneh //c++11 standard
    norm_est.setKSearch (n_neighbours);
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr get_normals(pcl::PointCloud<PointXYZRGB>::Ptr cloud){
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal> ());
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
    return normals;
  }
};

class Uniform{
public:
    pcl::UniformSampling<PointXYZRGB> uniform_sampling;
    pcl::PointCloud<int> sampled_indices;
    float cloud_ss_ ;

    void SetSamplingSize(float sampling_size){
        cloud_ss_ = sampling_size;
    }

    void GetKeypoints(pcl::PointCloud<PointXYZRGB>::Ptr cloud, pcl::PointCloud<PointXYZRGB>::Ptr cloud_keypoints){
        if (cloud_ss_ != 0){
            uniform_sampling.setInputCloud (cloud);
            uniform_sampling.setRadiusSearch (cloud_ss_);
            uniform_sampling.compute (sampled_indices);
            pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);
        }else
            std::cout << "no sampling size inserted" << std::endl;
    }

};

class Sift{
public:
    pcl::PointCloud<pcl::PointWithScale> cloud_result_;
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_;

    //pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_ ;//(new  KdTreeFLANN<pcl::PointXYZRGB> ());

    Sift():tree_(new pcl::search::KdTree<pcl::PointXYZRGB>()){//tree_(new  pcl::KdTreeFLANN<pcl::PointXYZRGB> ()){
        // Parameters for sift computation
        float min_scale(0.001);//0.001
        int n_octaves(6);
        int n_scales_per_octave (4);
        float min_contrast(0.1f);

        sift_.setSearchMethod(tree_);
        sift_.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift_.setMinimumContrast(min_contrast);
    }

    void GetKeypoints(pcl::PointCloud<PointXYZRGB>::Ptr cloud, pcl::PointCloud<PointXYZRGB>::Ptr cloud_keypoints) {

        sift_.setInputCloud(cloud);
        sift_.compute(cloud_result_);
        copyPointCloud(cloud_result_, *cloud_keypoints);
    }
};

Mat imageG;
int type_keypoint = 0;
pcl::PointCloud<PointXYZRGB>::Ptr model_keypoints (new pcl::PointCloud<PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointNormal>::Ptr model_normals (new pcl::PointCloud<pcl::PointNormal> ());

void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m);
void type_of_keypoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) ;
void Main_process();
pcl::PointCloud<pcl::SHOT1344>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT1344>());

int main(int argc, char **argv) {

    ros::init(argc, argv, "human_body_model");

    cout << "Enter your num of type :" << endl;
    cin >> type_keypoint;
    if (type_keypoint == 0) {
        cout << "enter your keypoint " << endl;
        exit(0);
    }
    ros::NodeHandle n_[2];
    ros::Subscriber sub1 = n_[0].subscribe("/camera/depth_registered/points", 1, Cloud_CallBack);

    image_transport::ImageTransport imageTransport(n_[1]);
    image_transport::Subscriber imageSubscriber;
    imageSubscriber = imageTransport.subscribe( "mythesis_person_image", 1, rosImageCallBack);
    boost::thread main_process(&Main_process);

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    main_process.interrupt();
    main_process.join();
    return 0;
}

void Main_process() {
    //    pcl::VoxelGrid<PointXYZRGB> VG_humanSampling;

    //    double clustering_voxel_size = 0.003;
    //    VG_humanSampling.setLeafSize (clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
    //    VG_humanSampling.setDownsampleAllData (false);

while (true) {
    
    if((imageG.size().width>0)&&(global_cloud->size()!=0))
    {
          person_cluster->clear();
          cv::Vec2b point;

        for(int i = 0; i<imageG.rows; ++i)
            for(int j = 0; j<imageG.cols; ++j )
            {
                point = imageG.at<cv::Vec2b>(i,j);
              if((point[0] == 0)&&(point[1] == 0)&&(point[2] == 0))
                  //&&(point[1] != 0)&&(point[2] != 0))
                {
                    person_cluster->points.push_back(global_cloud->at(j,i));
                }
            }
                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewerman"));
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(person_cluster);
                viewer1->addPointCloud<pcl::PointXYZRGB> (person_cluster, rgb, "sample cloud");
                while (!viewer1->wasStopped ()) {
                    viewer1->spinOnce (1);
                    boost::this_thread::sleep (boost::posix_time::microseconds (100));
                }
                viewer1.reset();

        imshow("windowerror", imageG);
        waitKey(3);
    

      // pcl::PointCloud<PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<PointXYZRGB>);
      // *cloud_ptr = *person_cluster;

      // if(cloud_ptr->size()>0)
      //   type_of_keypoint(cloud_ptr);

      //           // stringstream s2;
      //           // string root_path_model = ros::package::getPath("mythesis_body")+"/src/model_uniform/";
      //           // string endaddress = root_path_model; //ros::Pakeage::getPath(pick_and_place)+"src/Data/";
      //           // s2 << endaddress << "first" << "_shot.pcd";

      //           // pcl::io::savePCDFileBinary(s2.str(), *model_keypoints);



      // float descr_rad_(0.05);
      // // calculat shot descriptor 
      //   NormalEstimator norm;
      //   model_normals = norm.get_normals(person_cluster);//?
      //   //calculate descriptor
      //   pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, PointNormal, pcl::SHOT1344> est;
      //   est.setInputCloud(model_keypoints);
      //   est.setSearchSurface(person_cluster);//?doroste? bayad kole satho midadam ye faghat adamaro
      //   est.setInputNormals(model_normals);

      //   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      //   est.setSearchMethod(tree);
      //   est.setRadiusSearch (descr_rad_);
      //   est.compute (*model_descriptors);

      //   //****************************************************************
      //  cout << "*************keypoint size :  " << model_keypoints->size() << endl ;
      // //   temp_cluster.keypoint_size = model_keypoints->points.size();
      // //   temp_cluster.descriptors = model_descriptors->makeShared();
      // //   temp_cluster.objectName = "hichi";
      // //   scene_clusters.push_back(temp_cluster);

      // // save descriptor

      //           stringstream s2;
      //           string root_path_model = ros::package::getPath("mythesis_body")+"/src/model_uniform/";
      //           string endaddress = root_path_model; //ros::Pakeage::getPath(pick_and_place)+"src/Data/";
      //           s2 << endaddress << "descriptor second" << "_shot.pcd";

      //           pcl::io::savePCDFileBinary(s2.str(), *model_descriptors);

      }
    }

}
void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
{
    if (cloud_m->width != 0)
    {
        pcl::fromROSMsg(*cloud_m, *global_cloud);
    }
}
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr imagePointer;
    try
    {
        imagePointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    imageG = imagePointer->image;



    // if(imageG.size().width>0)
    // {
    //     for(int i = 0; i<imageG.rows; ++i)
    //         for(int j = 0; j<imageG.cols; ++j )
    //         {
    //             point = imageG.at<cv::Vec2b>(i,j);

    //             // cout<<"P0 " <<point[0]<<" p1 "<<point[1]<<" p2 "<<point[2]<<endl;
    //             if((point[0] == 0)&&(point[1] == 0)&&(point[2] == 0))
    //               //&&(point[1] != 0)&&(point[2] != 0))
    //             {
    //                 person_cluster->points.push_back(global_cloud->at(j,i));
    //             }
    //         }

    //             boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewerman"));
    //             // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //             pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(person_cluster);
    //             // // viewer->addCoordinateSystem (1.0, "global");
    //             // // viewer->initCameraParameters ();
    //             viewer1->addPointCloud<pcl::PointXYZRGB> (person_cluster, rgb, "sample cloud");
    //             while (!viewer1->wasStopped ()) {
    //                 viewer1->spinOnce (1);
    //                 boost::this_thread::sleep (boost::posix_time::microseconds (100));
    //             }
    //             viewer1.reset();


    // //     // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(person_cluster);
    // //     // viewer->addPointCloud<pcl::PointXYZRGB> (person_cluster, rgb, "sample cloud");
    // //    viewer->showCloud(person_cluster);
    // //    //  while (!viewer->wasStopped ()) {
    // //         //  viewer->spinOnce (1);
    // //         // boost::this_thread::sleep (boost::posix_time::microseconds (500));
    // //      // }
    // //     // viewer.reset();

    //     imshow("windowerror", imageG);
    //     waitKey(3);
    // }
}

void type_of_keypoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    float scene_ss_(0.005);  //scene sample size

    Sift sift_estimator;
    Uniform uniform;

    if (type_keypoint == 1) {
        //uniform
        uniform.SetSamplingSize(scene_ss_);
        uniform.GetKeypoints(cloud, model_keypoints);


    } /*else if (type_keypoint == 2) {
        //SIFT
        sift_estimator.GetKeypoints(cloud, model_keypoints);

    } else if (type_keypoint == 3) {
        //susan3D
        pcl::SUSANKeypoint<PointXYZRGB, PointXYZRGB> *susan3D = new pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
        susan3D->setInputCloud(cloud);
        susan3D->setNonMaxSupression(true);
        susan3D->compute(*  );

    } else if (type_keypoint == 4) {

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        double model_resolution = computeCloudResolution (cloud);
        // Compute model_resolution

        iss_salient_radius_ = 6 * model_resolution;
        iss_non_max_radius_ = 4 * model_resolution;
        iss_normal_radius_ = 4 * model_resolution;
        iss_border_radius_ = 1 * model_resolution;

        //
        // Compute keypoints
        //
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

        iss_detector.setSearchMethod (tree);
        iss_detector.setSalientRadius (iss_salient_radius_);
        iss_detector.setNonMaxRadius (iss_non_max_radius_);

        iss_detector.setNormalRadius (iss_normal_radius_);
        iss_detector.setBorderRadius (iss_border_radius_);

        iss_detector.setThreshold21 (iss_gamma_21_);
        iss_detector.setThreshold32 (iss_gamma_32_);
        iss_detector.setMinNeighbors (iss_min_neighbors_);
        iss_detector.setNumberOfThreads (iss_threads_);
        iss_detector.setInputCloud (cloud);
        iss_detector.compute (*model_keypoints);
    }*/
}
