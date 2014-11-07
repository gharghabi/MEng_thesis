//#include "openni2pcl_reg.hpp"
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



typedef pcl::PointXYZRGB PointType;
typedef pcl::PointNormal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef std::tuple<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >,std::vector<pcl::Correspondences>> ClusterType;


std::string model_filename;

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences(true);
bool use_hough_(true);
float model_ss_(0.005);  //model sample size
float scene_ss_(0.005);  //scene sample size
float rf_rad_(0.02);
float descr_rad_(0.05);
float cg_size_(0.007);
float cg_thresh_(6.0f);
bool narf(false);
bool sift(true); //if both narf and sift are false a uniform sampling is used with radius model_ss_ and scene_ss_
bool fpfh(false); // if false shot descriptors are used
bool ransac(false);
const int distance = 700; //kinect cut-off distance
 // Parameters for sift computation
float min_scale(0.001);//0.001
int n_octaves(6);
int n_scales_per_octave (4);
float min_contrast(0.1f);
// parameters for narf computation
float support_size(0.02);

void showHelp (char *filename){

  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Real time object recognition - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -show_keypoints:                     Show used keypoints." << std::endl;
  std::cout << "     -show_correspondences:                     Show used correspondences." << std::endl;
  std::cout << "     --algorithm (hough|gc): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --keypoints (narf|sift|uniform): Keypoints detection algorithm (default uniform)." << std::endl;
  std::cout << "     --descriptors (shot|fpfh): Descriptor type (default shot)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.005)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.005)" << std::endl;
  std::cout << "     --rf_rad val:           Hough reference frame radius (default 0.02)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.03)" << std::endl;
  std::cout << "     --cg_size val:          Dimension of Hough's bins (default 0.007)" << std::endl;
  std::cout << "     --cg_thresh val:        Minimum number of positive votes for a match (default 6)" << std::endl << std::endl;
  std::cout << "     --sift_min_scale:       (default 0.001)" << std::endl;
  std::cout << "     --sift_octaves:         (default 6)" << std::endl;
  std::cout << "     --sift_scales_per_octave:  (default 4)" << std::endl;
  std::cout << "     --sift_min_contrast:    (default 0.3)" << std::endl << std::endl;
  std::cout << "     --narf_support_size:    (default 0.02)" << std::endl << std::endl;
 

}

void parseCommandLine (int argc, char *argv[]){
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  //std::vector<int> filenames;
  //filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//  if (filenames.size () != 1)
//  {
//    std::cout << "Filenames missing.\n";
//    showHelp (argv[0]);
//    exit (-1);
//  }

  //model_filename = argv[filenames[0]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-show_keypoints"))
    show_keypoints_ = true;
  if (pcl::console::find_switch (argc, argv, "-show_correspondences"))
    show_correspondences = true;
 
  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("hough") == 0)
      use_hough_ = true;
    else if (used_algorithm.compare ("gc") == 0)
      use_hough_ = false;
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  std::string used_keypoints;
  if (pcl::console::parse_argument (argc, argv, "--keypoints", used_keypoints) != -1)
  {
    if (used_keypoints.compare ("narf") == 0)
      narf = true;
    else if (used_keypoints.compare ("sift") == 0)
      sift = true;
    else if(used_keypoints.compare ("ransac") == 0)
      ransac = true;
    else if(used_keypoints.compare ("uniform") == 0)
      std::cout << "Using uniform sampling.\n";
    
  }

  std::string used_descriptors;
  if (pcl::console::parse_argument (argc, argv, "--descriptors", used_descriptors) != -1)
  {
    if (used_descriptors.compare ("shot") == 0)
      fpfh = false;
    else if (used_descriptors.compare ("fpfh") == 0)
      fpfh = true;
    else
    {
      std::cout << "Wrong descriptors type .\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
  pcl::console::parse_argument (argc, argv, "--sift_min_scale", min_scale);
  pcl::console::parse_argument (argc, argv, "--sift_octaves", n_octaves);
  pcl::console::parse_argument (argc, argv, "--sift_scales_per_octave", n_scales_per_octave);
  pcl::console::parse_argument (argc, argv, "--sift_min_contrast", min_contrast);
  pcl::console::parse_argument (argc, argv, "--narf_support_size", support_size);
}

void SetViewPoint(pcl::PointCloud<PointType>::Ptr cloud){

    cloud->sensor_origin_.setZero();
    cloud->sensor_orientation_.w () = 0.0;
    cloud->sensor_orientation_.x () = 1.0;
    cloud->sensor_orientation_.y () = 0.0;
    cloud->sensor_orientation_.z () = 0.0;
}

void PrintTransformation(ClusterType cluster){
  for (size_t i = 0; i < std::get<0>(cluster).size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << std::get<1>(cluster)[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = std::get<0>(cluster)[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = std::get<0>(cluster)[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
}



template <class T, class Estimator>
class KeyDes{
public:
  typedef pcl::PointCloud<T> PD;
  typedef pcl::PointCloud<PointType> P;
  typedef pcl::PointCloud<NormalType> PN;
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


class NormalEstimator{
public:
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;

  NormalEstimator() {
    norm_est.setKSearch (10);
  }

  NormalEstimator(int n_neighbours){//: NormalEstimator() {//ameneh //c++11 standard
    norm_est.setKSearch (n_neighbours);
  }

  pcl::PointCloud<NormalType>::Ptr get_normals(pcl::PointCloud<PointType>::Ptr cloud){
    pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType> ());
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
    return normals;
  }
};


class DownSampler{
public:
  pcl::VoxelGrid<pcl::PointXYZRGB> down_sampler_;

  DownSampler() {
    down_sampler_.setLeafSize (1.5, 1.5, 1.5);
  }

  DownSampler(float x, float y, float z){
    down_sampler_.setLeafSize(x, y, z);
  }

  void SetSampleSize(float x, float y, float z){
    down_sampler_.setLeafSize(x, y, z);
  }

  void DownSample(pcl::PointCloud<PointType>::Ptr cloud){
    down_sampler_.setInputCloud(cloud);
    down_sampler_.filter(*cloud);
  }
};


class Narf{
public:
  pcl::PointCloud<int> cloud_keypoint_indices_;
  Eigen::Affine3f cloud_sensor_pose_;
  bool rotation_invariant_;
  pcl::RangeImageBorderExtractor range_image_border_extractor_;
  pcl::NarfKeypoint narf_keypoint_detector_;




  Narf(): rotation_invariant_(true), cloud_sensor_pose_(Eigen::Affine3f::Identity ()) {
    narf_keypoint_detector_.setRangeImageBorderExtractor (&range_image_border_extractor_);
    narf_keypoint_detector_.getParameters().support_size = support_size;

  }

  void GetKeypoints(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints){

    boost::shared_ptr<pcl::RangeImage>cloud_range_image_ptr_(new pcl::RangeImage);

    cloud_sensor_pose_ = Eigen::Affine3f (Eigen::Translation3f (cloud->sensor_origin_[0],
                                                                cloud->sensor_origin_[1],
                                                                cloud->sensor_origin_[2])) *
                                                                Eigen::Affine3f (cloud->sensor_orientation_);

    pcl::RangeImage& cloud_range_image_ = *cloud_range_image_ptr_;

    narf_keypoint_detector_.setRangeImage (&cloud_range_image_);
    

    cloud_range_image_.createFromPointCloud(*cloud, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                             cloud_sensor_pose_, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
    
    cloud_range_image_.setUnseenToMaxRange();
    
    narf_keypoint_detector_.compute(cloud_keypoint_indices_);
    
    cloud_keypoints->points.resize (cloud_keypoint_indices_.points.size ());
    
    #pragma omp parallel for
    for (size_t i=0; i<cloud_keypoint_indices_.points.size (); ++i)
      cloud_keypoints->points[i].getVector3fMap () = cloud_range_image_.points[cloud_keypoint_indices_.points[i]].getVector3fMap ();
  }
};


class Sift{
public:
  pcl::PointCloud<pcl::PointWithScale> cloud_result_;
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_;

  //pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_ ;//(new  KdTreeFLANN<pcl::PointXYZRGB> ());

  Sift():tree_(new pcl::search::KdTree<pcl::PointXYZRGB>()){//tree_(new  pcl::KdTreeFLANN<pcl::PointXYZRGB> ()){
    sift_.setSearchMethod(tree_);
    sift_.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift_.setMinimumContrast(min_contrast);
  }

  void GetKeypoints(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints) {

    sift_.setInputCloud(cloud);
    sift_.compute(cloud_result_);
    copyPointCloud(cloud_result_, *cloud_keypoints);
  }
};


template <class T>
class Ransac{
public:
  std::vector<int> cloud_inliers;

  void GetKeypoints(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints){

  typename T::Ptr cloud_plane (new T (cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> model_ransac (cloud_plane);
  model_ransac.computeModel();
  model_ransac.getInliers(cloud_inliers);

  pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, cloud_inliers, *cloud_keypoints);
  }

};


class Uniform{
public:
  pcl::UniformSampling<PointType> uniform_sampling;
  pcl::PointCloud<int> sampled_indices;
  float cloud_ss_ ;

  void SetSamplingSize(float sampling_size){
    cloud_ss_ = sampling_size;
  }

  void GetKeypoints(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_keypoints){
    if (cloud_ss_ != 0){
      uniform_sampling.setInputCloud (cloud);
      uniform_sampling.setRadiusSearch (cloud_ss_);
      uniform_sampling.compute (sampled_indices);
      pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);
    }else
      std::cout << "no sampling size inserted" << std::endl;
  }

};


class Hough{
public:
  ClusterType cluster;
  pcl::PointCloud<RFType>::Ptr model_rf_;
  pcl::PointCloud<RFType>::Ptr scene_rf_;
  pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est_;
  pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer_;
  bool created;
  


  Hough(): model_rf_(new pcl::PointCloud<RFType> ()), scene_rf_(new pcl::PointCloud<RFType> ()), created(false) {
    rf_est_.setFindHoles (true);
    rf_est_.setRadiusSearch (rf_rad_);
    clusterer_.setHoughBinSize (cg_size_);
    clusterer_.setHoughThreshold (cg_thresh_);
    clusterer_.setUseInterpolation (true);
    clusterer_.setUseDistanceWeight (false);

  }



  ClusterType GetClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints,
                   pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints, pcl::PointCloud<NormalType>::Ptr scene_normals,
                    pcl::CorrespondencesPtr model_scene_corrs){
    //  Compute (Keypoints) Reference Frames only for Hough
    if(!created){
      rf_est_.setInputCloud (model_keypoints);
      rf_est_.setInputNormals (model_normals);
      rf_est_.setSearchSurface (model);
      rf_est_.compute (*model_rf_);
    }

    //std::cout << "computed hough BOARD on model" <<std::endl;

    rf_est_.setInputCloud (scene_keypoints);
    rf_est_.setInputNormals (scene_normals);
    rf_est_.setSearchSurface (scene);
    rf_est_.compute (*scene_rf_);

    //std::cout << "computed hough BOARD on scene" <<std::endl;

    //  Clustering
    if(!created){
      clusterer_.setInputCloud (model_keypoints);
      clusterer_.setInputRf (model_rf_);
      created = true;
    }

    clusterer_.setSceneCloud (scene_keypoints);
    clusterer_.setSceneRf (scene_rf_);
    clusterer_.setModelSceneCorrespondences (model_scene_corrs);

    //std::cout << "prepared Hough for clustering" <<std::endl;

    //clusterer_.cluster (clustered_corrs);
    clusterer_.recognize (std::get<0>(cluster), std::get<1>(cluster));
    return cluster;
  }

};

class GCG{
public:
  pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer_;
  ClusterType cluster;

  GCG(){
    gc_clusterer_.setGCSize (cg_size_);
    gc_clusterer_.setGCThreshold (cg_thresh_);
  }


  ClusterType GetClusters(pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints, pcl::CorrespondencesPtr model_scene_corrs){

    gc_clusterer_.setInputCloud (model_keypoints);
    gc_clusterer_.setSceneCloud (scene_keypoints);
    gc_clusterer_.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer_.cluster (clustered_corrs);
    gc_clusterer_.recognize (std::get<0>(cluster), std::get<1>(cluster));
    return cluster;

  }
};

class Visualizer{
public:
  pcl::visualization::PCLVisualizer viewer_;
  pcl::PointCloud<PointType>::Ptr off_scene_model_;
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints_;
  bool to_stop;
  int iter;

  Visualizer(): off_scene_model_(new pcl::PointCloud<PointType> ()), off_scene_model_keypoints_(new pcl::PointCloud<PointType> ()), to_stop(false), iter(0){}

  void Visualize(pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene,
                 pcl::PointCloud<PointType>::Ptr scene_keypoints, ClusterType cluster){

    if (viewer_.wasStopped ())
      to_stop = true;


    SetViewPoint(scene);
    if(iter == 0)
      viewer_.addPointCloud (scene, "scene_cloud");
    else
      viewer_.updatePointCloud (scene, "scene_cloud");


    if (show_correspondences && iter == 0){
      //  We are translating the model so that it doesn't end in the middle of the scene representation
      pcl::transformPointCloud (*model, *off_scene_model_, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints_, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

      SetViewPoint(off_scene_model_);
      SetViewPoint(off_scene_model_keypoints_);

      //pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model__color_handler (off_scene_model_, 255, 255, 128);
      viewer_.addPointCloud (off_scene_model_,  "off_scene_model_");
    }

    if (show_keypoints_){
      pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
      SetViewPoint(scene_keypoints);
      if(iter == 0)
        viewer_.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
      else
        viewer_.updatePointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
      viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints__color_handler (off_scene_model_keypoints_, 0, 0, 255);
      SetViewPoint(off_scene_model_keypoints_);
      if(iter == 0)
        viewer_.addPointCloud (off_scene_model_keypoints_, off_scene_model_keypoints__color_handler, "off_scene_model_keypoints_");
      else
        viewer_.updatePointCloud (off_scene_model_keypoints_, off_scene_model_keypoints__color_handler, "off_scene_model_keypoints_");
      viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints_");
    }

    for (size_t i = 0; i < std::get<0>(cluster).size (); ++i){
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *rotated_model, std::get<0>(cluster)[i]);
      SetViewPoint(rotated_model);

      std::stringstream ss_cloud;
      ss_cloud << "instance" << i;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
      if(iter == 0)
        viewer_.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
      else
        viewer_.updatePointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
      /*
      if (show_correspondences){
        for (size_t j = 0; j < std::get<1>(cluster)[i].size (); ++j){
          std::stringstream ss_line;
          ss_line << "correspondence_line" << i << "_" << j << "_" << iter;
          PointType& model_point = off_scene_model_keypoints_->at (std::get<1>(cluster)[i][j].index_query);
          PointType& scene_point = scene_keypoints->at (std::get<1>(cluster)[i][j].index_match);

          //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
          viewer_.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }
      }*/
    }
    viewer_.spinOnce();
  iter++;
  }
};
