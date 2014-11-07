#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/filesystem.hpp>
#include <pcl/features/vfh.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;


std::vector<string> models;
ofstream OFS;
int correctDetect,allDetectionNum;
template <class T, class Estimator>
class KeyDes{
public:
    typedef pcl::PointCloud<T> PD;
    typedef pcl::PointCloud<pcl::PointXYZRGB> P;
    typedef pcl::PointCloud<pcl::PointNormal> PN;
    typename PD::Ptr model_descriptors;
    typename PD::Ptr scene_descriptors;

    //KeyDes(P::Ptr model, P::Ptr model_keypoints, P::Ptr scene, P::Ptr scene_keypoints, PN::Ptr model_normals, PN::Ptr scene_normals ):
    KeyDes(PD model_d,PD scene_d):

        model_descriptors (new PD (model_d)),
        scene_descriptors (new PD (scene_d)){}
    pcl::CorrespondencesPtr run()
    {
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
        pcl::KdTreeFLANN<T> match_search;
        //  Find Model-Scene Correspondences with KdTree
//        std::cout << "calculating correspondences "  <<std::endl;

        match_search.setInputCloud (model_descriptors);

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
#pragma omp parallel for
        for (size_t i = 0; i < scene_descriptors->size (); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);

            int found_neighs = match_search.nearestKSearch (scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            OFS<<" dist: "<<neigh_sqr_dists[0];
            if(found_neighs == 1) //  0.25 std add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
              pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
              #pragma omp critical
              model_scene_corrs->push_back (corr);
            }
          }
//          std::cout << "\tFound "  <<model_scene_corrs->size ()<< " correspondences "<< std::endl;
          return model_scene_corrs;
    }
};


bool loadFileList (std::vector<string> &models, const std::string &filename);
void findPcdModelsInFolder (const boost::filesystem::path &base_dir, const std::string &extension);
pcl::PointCloud<pcl::VFHSignature308>::Ptr extract_feature (const std::string &filename);
void calc_correspondense( pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHfeature,const std::string &filename);
int main(int argc, char **argv) {

    ros::init(argc, argv, "testVFHOnDataset_node");
    string trainingDataListAdress = ros::package::getPath("mythesis_body")+"/src/training_data_vfh_pcd.list";
    string fileaddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/walking2/";
    string extension (".pcd");

    string correspondenseResult = ros::package::getPath("mythesis_body")+"/src/correspondense_vfh_walking1_walking2.txt";
    OFS.open (correspondenseResult.c_str ());
    loadFileList(models,trainingDataListAdress);
    findPcdModelsInFolder (fileaddress, extension);
    OFS.close();
    cout<< " darsade tashkhis "<<correctDetect<<" tedade kol "<<allDetectionNum<<endl;
}
bool loadFileList (std::vector<string> &models, const std::string &filename)
{
    ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
        stringstream s;
        s<<filename.c_str()<<"  dosen't exist";
        return (false);
    }

    std::string line;
    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.empty ())
        {
            continue;
        }
        string m;
        m = line;
        models.push_back (m);
    }
    fs.close ();
    return (true);
}
void findPcdModelsInFolder (const boost::filesystem::path &base_dir, const std::string &extension)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it) {
        if (boost::filesystem::is_directory (it->status ())) {
            stringstream ss;
            ss << it->path ();
            //            pcl::console::print_highlight ("Loading %s .\n", ss.str ().c_str ());
            findPcdModelsInFolder (it->path (), extension);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension) {
            string m;
            boost::filesystem::path path;
            path=base_dir / it->path ().filename ();
            m=path.string();
            ROS_INFO(m.c_str());
            // extract features
            pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHfeature = extract_feature (m);
            calc_correspondense(VFHfeature,m);
        }
    }
}
pcl::PointCloud<pcl::VFHSignature308>::Ptr extract_feature (const std::string &filename)
{
    TicToc tt;
    tt.tic ();

    //    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (filename, *cloud);
    cloud->width = (int) cloud->points.size ();
    cloud->height = 1;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree_normal);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (15);
    ne.compute (*cloud_normals);

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

    return vfhs;
    //    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->width * cloud->height); print_info (" points]\n");
}
void calc_correspondense( pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHfeature,const std::string &filename)
{
    pcl::PointCloud <pcl::VFHSignature308>::Ptr point_vfh(new pcl::PointCloud<pcl::VFHSignature308> ());;
    string normal_path = "walking2 ";
    std::vector<std::string> strs;
    boost::algorithm::split(strs,filename,boost::algorithm::is_any_of("/"));
    std::string person_num = strs.at((strs.size()-3));
    std::string model_num = strs.at((strs.size()-1));
    normal_path.append(person_num);
    normal_path.append(model_num);

    float minDist = 10000;
    string minDistCloudName;
    for (int i = 0; i < models.size(); ++i) {
        string cloud_name = models[i] ;
        if (pcl::io::loadPCDFile (cloud_name, *point_vfh) < 0)
        {
            cout << "error" << endl;
            //break;
        } else
        {
            OFS<<"input: "<<normal_path;

            std::vector<std::string> strsModel;
            boost::algorithm::split(strsModel,cloud_name,boost::algorithm::is_any_of("/"));

            OFS<<" model: "<<strsModel.at((strsModel.size()-1));
            pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
            KeyDes<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> > est (*VFHfeature, *point_vfh);
            model_scene_corrs = est.run();

            if(model_scene_corrs->at(0).distance<minDist)
            {
                minDist = model_scene_corrs->at(0).distance;
                minDistCloudName = strsModel.at((strsModel.size()-1));
            }
            OFS<<" corres size: "<<model_scene_corrs->size()<<" corres dist: "<< model_scene_corrs->at(0).distance<<"\n";

        }
    }
    if((minDistCloudName.at(0)==normal_path.at(9))&&(minDistCloudName.at(1)==normal_path.at(10)))
    {
        ++correctDetect;
    }
    OFS<<" moshabeh taring: "<<minDistCloudName<<"\n";
    ++allDetectionNum;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::io::loadPCDFile ("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2.pcd", *cloud);
//cloud->width = (int) cloud->points.size ();
//cloud->height = 1;

//// Create the normal estimation class, and pass the input dataset to it
//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//ne.setInputCloud (cloud);
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
//ne.setSearchMethod (tree_normal);
//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//ne.setRadiusSearch (30);
//ne.compute (*cloud_normals);

//pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2normalcodeCorrR30.pcd", *cloud_normals);

//// Create the VFH estimation class, and pass the input dataset+normals to it
//pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
//vfh.setInputCloud (cloud);
//vfh.setInputNormals (cloud_normals);

//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//vfh.setSearchMethod (tree);

//// Output datasets
//pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

//// Compute the features
//vfh.compute (*vfhs);
//pcl::io::savePCDFileASCII("/home/shaghayegh/catkin_ws/src/mythesis_body/1_2vfhcodeCorresR30.pcd", *vfhs);

//pcl::PointCloud <pcl::VFHSignature308>::Ptr point_vfh(new pcl::PointCloud<pcl::VFHSignature308> ());;
//pcl::io::loadPCDFile("/home/shaghayegh/catkin_ws/src/mythesis_body/31vfhcodeCorresR30.pcd", *point_vfh);


//pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

//KeyDes<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> > est (*vfhs, *vfhs);
//model_scene_corrs = est.run();
//cout<< "size "<<model_scene_corrs->size()<<endl;


//pcl::CorrespondencesPtr model_scene_corrs_2 (new pcl::Correspondences ());

//KeyDes<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> > est2 (*vfhs, *point_vfh);
//model_scene_corrs_2 = est2.run();
//cout<< "size2 "<<model_scene_corrs_2->size()<<endl;
