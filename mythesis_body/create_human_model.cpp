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
#include <pcl/search/octree.h>
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
    // norm_est.setKSearch (10);
        norm_est.setRadiusSearch (9);
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
        float min_scale(1);//0.001
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
pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());

void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m);
void type_of_keypoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) ;
double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
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

          person_cluster->clear();
          cv::Vec2b point;

        // for(int i = 0; i<imageG.rows; ++i)
        //     for(int j = 0; j<imageG.cols; ++j )
        //     {
        //         point = imageG.at<cv::Vec2b>(i,j);
        //       if((point[0] == 0)&&(point[1] == 0)&&(point[2] == 0))
        //           //&&(point[1] != 0)&&(point[2] != 0))
        //         {
        //             person_cluster->points.push_back(global_cloud->at(j,i));
        //         }
        //     }

      pcl::PointCloud<PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<PointXYZRGB>);
              pcl::PCDReader reader;
			  reader.read<pcl::PointXYZRGB> ("/home/shaghayegh/catkin_ws/src/mythesis_body/01.pcd", *cloud_ptr);
			  	// reader.read<pcl::PointXYZRGB> ("/home/shaghayegh/catkin_ws/src/mythesis_body/src/model_uniform/third_shot.pcd", *person_cluster);
              // pcl::io::savePCDFile("/home/shaghayegh/catkin_ws/pcdmamuli.pcd",*person_cluster);
              // pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/pcdascii.pcd",*person_cluster);
              // pcl::io::savePCDFileBinary("/home/shaghayegh/catkin_ws/pcdBinary.pcd",*person_cluster);

    //           for (size_t i = 0; i < person_cluster->points.size (); ++i)
    // std::cout << "    " << person_cluster->points[i].x
    //           << " "    << person_cluster->points[i].y
    //           << " "    << person_cluster->points[i].z << std::endl;
			  	cout<<"model_size "<<cloud_ptr->points.size()<<endl;
                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewerman"));
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
                viewer1->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, rgb, "sample cloud");
                while (!viewer1->wasStopped ()) {
                    viewer1->spinOnce (1);
                    boost::this_thread::sleep (boost::posix_time::microseconds (100));
                }
                viewer1.reset();

    
      pcl::PointCloud<PointXYZRGB>::Ptr cloud_ptr2 (new pcl::PointCloud<PointXYZRGB>);

      *person_cluster = *cloud_ptr->makeShared();
      cout <<cloud_ptr->isOrganized()<<" organized "<<endl;
      cout<<cloud_ptr->header << " header "<<endl;
      waitKey(0);

      if(cloud_ptr->size()>0)
      {

          // std::vector<int> indices;
          // pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud_ptr2, indices);
          // std::cout << "size: " << cloud_ptr2->points.size () << std::endl;//          pcl::removeNaNFromPointCloud()
        type_of_keypoint(cloud_ptr);
      }
    	ROS_INFO(" type of keypoint ");
                stringstream s2;
                string root_path_model = ros::package::getPath("mythesis_body")+"/src/model_uniform/";
                string endaddress = root_path_model; //ros::Pakeage::getPath(pick_and_place)+"src/Data/";
                s2 << endaddress << "third" << "_shot.pcd";
                pcl::io::savePCDFileBinary("/home/shaghayegh/catkin_ws/src/mythesis_body/01_uniform.pcd", *model_keypoints);
       cout << "*************keypoint size :  " << model_keypoints->size() << endl ;



      float descr_rad_(0.5);
      // calculat shot descriptor 
//        NormalEstimator norm;
//        model_normals = norm.get_normals(cloud_ptr);//?
//        pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/01_shot_normalR9.pcd", *model_normals);


      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud (cloud_ptr);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree_normal);
//      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (9);
      ne.compute (*model_normals);
                pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/01_shot_normalR9.pcd", *model_normals);

      //         if(cloud_ptr->size()>0)
      // {
      //     std::vector<int> indices;
      //     pcl::removeNaNFromPointCloud(*person_cluster, *cloud_ptr2, indices);
      //     std::cout << "size: " << cloud_ptr2->points.size () << std::endl;//          pcl::removeNaNFromPointCloud()
      // }

     
        //calculate descriptor
        pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, PointNormal, pcl::SHOT1344> est;
        est.setInputCloud(model_keypoints);
        est.setSearchSurface(person_cluster);//?doroste? bayad kole satho midadam ye faghat adamaro
        est.setInputNormals(model_normals);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        est.setSearchMethod(tree);
        est.setRadiusSearch (descr_rad_);
        est.compute (*model_descriptors);

        //****************************************************************
               cout << "*************key_points size :  " << model_keypoints->size() << endl ;

       cout << "*************descriptor size :  " << model_descriptors->size() << endl ;
      //   temp_cluster.keypoint_size = model_keypoints->points.size();
      //   temp_cluster.descriptors = model_descriptors->makeShared();
      //   temp_cluster.objectName = "hichi";
      //   scene_clusters.push_back(temp_cluster);

      // save descriptor

                stringstream s3;
                string root_path_model_descriptor = ros::package::getPath("mythesis_body")+"/src/model_uniform/";
                string endaddress_descriptor = root_path_model_descriptor; //ros::Pakeage::getPath(pick_and_place)+"src/Data/";
                s3 << endaddress_descriptor << "descriptor second" << "_shot.pcd";

                pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/01_shot_ShotnormalR9.pcd", *model_descriptors);


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
    float model_ss_(100);//0.005  //scene sample size

    Sift sift_estimator;
    Uniform uniform;

    if (type_keypoint == 1) {//ye file pcd misaze ke tamame dadeshash yeksanan nazdike 30000000 ham khat dare jalebe ke size keypointo 9099 mide
        //uniform
        uniform.SetSamplingSize(model_ss_);
        uniform.GetKeypoints(cloud, model_keypoints);


    } else if (type_keypoint == 2) {//khalie keypoint haye hesab kardash //[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow
        //SIFT
        sift_estimator.GetKeypoints(cloud, model_keypoints);

    } else if (type_keypoint == 3) {//khalie keypoint haye hesab kardash
        //susan3D
        pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> *susan3D = new pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
        susan3D->setInputCloud(cloud);
        susan3D->setNonMaxSupression(true);
        susan3D->compute(*model_keypoints);

    }else if (type_keypoint == 4) {//key point hesab shod shot error mide [pcl::SHOTColorEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 73
//gofte 146 ta keypoint hesab karde ke maskharast chon filesho ke baz karam ye satr sefr tush por bud
		double iss_salient_radius_;
		double iss_non_max_radius_;
		double iss_normal_radius_;
		double iss_border_radius_;
		double iss_gamma_21_ (0.975);
		double iss_gamma_32_ (0.975);
		double iss_min_neighbors_ (5);
		int iss_threads_ (4);

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
    }
}
double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i) {
        if (! pcl_isfinite ((*cloud)[i].x)) {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    return res;
}
