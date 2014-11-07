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
#include <pcl/features/vfh.h>
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
#include <pcl/range_image/range_image_planar.h>
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
#include <pcl/features/fpfh_omp.h>
#include <pcl/range_image/range_image.h>
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
#include <upperbodycore_msgs/coordinate.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
//jost for this code
//#include "pcl_object_recognition.hpp"
#include <upperbodycore_msgs/nameobjects.h>
#include <upperbodycore_msgs/objectsposition.h>
#include <upperbodycore_msgs/objectpos.h>
#include <upperbodycore_msgs/setthreshSrv.h>
#include <upperbodycore_msgs/run_objectrecognitionSrv.h>
#include <upperbodycore_msgs/ack_packages.h>
#include <upperbodycore_msgs/log_Packages.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <ros/package.h>

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
template <class T, class Estimator>
class KeyDes{
public:
  typedef pcl::PointCloud<T> PD;
  typedef pcl::PointCloud<pcl::PointXYZRGB> P;
  typedef pcl::PointCloud<pcl::PointNormal> PN;
  typename PD::Ptr model_descriptors;
  typename PD::Ptr scene_descriptors;
  //typename P::Ptr model ;
  //typename P::Ptr model_keypoints;
 // typename P::Ptr scene;
 // typename P::Ptr scene_keypoints;
 // typename PN::Ptr model_normals;
 // typename PN::Ptr scene_normals;
 // bool created;


 //KeyDes(P::Ptr model, P::Ptr model_keypoints, P::Ptr scene, P::Ptr scene_keypoints, PN::Ptr model_normals, PN::Ptr scene_normals ):
 KeyDes(PD model_d,PD scene_d):

            model_descriptors (new PD (model_d)),
            scene_descriptors (new PD (scene_d)){}
//            model(model),
//            model_keypoints(model_keypoints),
//            scene(scene),
//            scene_keypoints(scene_keypoints),
//            model_normals(model_normals),
//            scene_normals(scene_normals){}
         // created(false){}

  pcl::CorrespondencesPtr run()
  {

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    //create scene descriptors
//    std::cout << "calculating scene descriptors "  <<std::endl;
//    Estimator est;
//    est.setInputCloud(scene_keypoints);
//    est.setSearchSurface(scene);
//    est.setInputNormals(scene_normals);

//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//    est.setSearchMethod(tree);
//    est.setRadiusSearch (descr_rad_);
//    est.compute (*scene_descriptors);

//    if(!created){
//      //create model descriptors
//      std::cout << "calculating model descriptors "  <<std::endl;
//      std::clock_t start;
//         double duration;
//         start = std::clock();
//      est.setInputCloud(model_keypoints);
//      est.setSearchSurface(model);
//      est.setInputNormals(model_normals);
//      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
//      est.setSearchMethod(tree2);
//      est.compute (*model_descriptors);
//      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

//      std::cout<<"time 2: "<< duration <<'\n';
//      created = true;
//  }

    pcl::KdTreeFLANN<T> match_search;
    //std::cout <<"calculated " << model_descriptors->size() << " for the model and " << scene_descriptors->size() << " for the scene" <<std::endl;

    //  Find Model-Scene Correspondences with KdTree
    std::cout << "calculating correspondences "  <<std::endl;

    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    #pragma omp parallel for
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);

      int found_neighs = match_search.nearestKSearch (scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
      cout<<neigh_sqr_dists[0]<<" dist "<<endl;
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  0.25 std add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);

        #pragma omp critical
        model_scene_corrs->push_back (corr);
      }
    }
    std::cout << "\tFound "  <<model_scene_corrs->size ()<< " correspondences "<< std::endl;
    return model_scene_corrs;

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
pcl::PointCloud<pcl::PointNormal>::Ptr model_normals (new pcl::PointCloud<pcl::PointNormal> ());

void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m);
void type_of_keypoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) ;
double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
void Main_process();
pcl::PointCloud<pcl::SHOT1344>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT1344>());

int main(int argc, char **argv) {

    ros::init(argc, argv, "human_body_model");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2.pcd", *cloud);
  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree_normal);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (30);
  ne.compute (*cloud_normals);

  pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2normalcodeCorrR30.pcd", *cloud_normals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);
  pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2vfhcodeCorresR30.pcd", *vfhs);

  pcl::PointCloud <pcl::VFHSignature308>::Ptr point_vfh(new pcl::PointCloud<pcl::VFHSignature308> ());;
  pcl::io::loadPCDFile("/home/shaghayegh/catkin_ws/src/mythesis_body/31vfhcodeCorresR30.pcd", *point_vfh);


  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  KeyDes<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> > est (*vfhs, *vfhs);
  model_scene_corrs = est.run();
cout<< "size "<<model_scene_corrs->size()<<endl;


pcl::CorrespondencesPtr model_scene_corrs_2 (new pcl::Correspondences ());

KeyDes<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> > est2 (*vfhs, *point_vfh);
model_scene_corrs_2 = est2.run();
cout<< "size2 "<<model_scene_corrs_2->size()<<endl;
//  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

//  KeyDes<pcl::SHOT1344, pcl::VFHSignature308<pcl::PointXYZ,  pcl::Normal, pcl::> > est (*model, *scene_clusters[j].descriptors);
//  model_scene_corrs = est.run();

}
