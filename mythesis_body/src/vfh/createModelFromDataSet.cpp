
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

ofstream fs;
void printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.ply output.pcd\n", argv[0]);
}

void extract_feature (const std::string &filename)
{
    TicToc tt;
    tt.tic ();

    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

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

    string normal_path = ros::package::getPath("mythesis_body")+"/dataset/RGBD-ID/data/vfh_normals/";
    std::vector<std::string> strs;
    boost::algorithm::split(strs,filename,boost::algorithm::is_any_of("/"));
    std::string person_num = strs.at((strs.size()-3));
    std::string model_num = strs.at((strs.size()-1));
    normal_path.append(person_num);
    normal_path.append(model_num);
    pcl::io::savePCDFileASCII(normal_path, *cloud_normals);

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

    string vfhModlePath = ros::package::getPath("mythesis_body")+"/dataset/RGBD-ID/data/vfh_model/";
    vfhModlePath.append(person_num);
    vfhModlePath.append(model_num);

    pcl::io::savePCDFileASCII(vfhModlePath, *vfhs);
    fs << vfhModlePath  << "\n";

    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->width * cloud->height); print_info (" points]\n");

}

/* ---[ */
void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it) {
        if (boost::filesystem::is_directory (it->status ())) {
            stringstream ss;
            ss << it->path ();
            pcl::console::print_highlight ("Loading %s .\n", ss.str ().c_str ());
            loadFeatureModels (it->path (), extension);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension) {
            string m;
            boost::filesystem::path path;
            path=base_dir / it->path ().filename ();
            m=path.string();
            // extract features
            extract_feature (m);
        }
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "createModelFromDataSet_node");
    print_info ("Create vfh model.\n");
    string fileaddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/walking1/";
    string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    string trainingDataListAdress = ros::package::getPath("mythesis_body")+"/src/training_data_vfh_pcd.list";
    fs.open (trainingDataListAdress.c_str ());
    // Load the model histograms
    loadFeatureModels (fileaddress, extension);
    fs.close();
}

