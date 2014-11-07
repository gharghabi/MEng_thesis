#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include "calc_features.h"
#include <pcl/console/time.h>
#include <fstream>
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

using namespace std;
std::ofstream OFTrainData,OFTestData;
std::ofstream OFCorrespondece;

struct features
{
    double d[13];
    string model;
    int aveNum;//tedade vijegi hayi ke az an ha miangin gerefte shode ast
};
int featuresCount = 8;
double ave_var[13][2];
calc_features *gClient;
std::vector<features> testModels,trainModels;
bool loadTrainModels(std::vector<features> &trainModels, const std::string filename);
void loadtestModels (std::vector<features> &testModels, const std::string filename);
void calcFeatureMatric(std::vector<features> &trainModels,std::vector<features> &testModels);
void calcFeatureMatricRankK(std::vector<features> &trainModels,std::vector<features> &testModels);
upperbodycore_msgs::Skeleton extract_feature (const std::string &filename);
//void calc_correspondense( pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHfeature,const std::string &filename);
int main(int argc, char **argv) {

    ros::init(argc, argv, "testVFHOnDataset_node");

    gClient = new calc_features();
    string fileaddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/walking2/";
    string trainFileAddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/collaborative/";
    string Trainfeatures = ros::package::getPath("mythesis_body")+"/src/collaborativeFeatures.txt";
    string Testfeatures = ros::package::getPath("mythesis_body")+"/src/walking2Features.txt";
    string correspondenseResult = ros::package::getPath("mythesis_body")+"/src/correspondenceFeatures.txt";
    OFCorrespondece.open (correspondenseResult.c_str ());
    OFTrainData.open (Trainfeatures.c_str());
    OFTestData.open(Testfeatures.c_str());
    loadTrainModels(trainModels,trainFileAddress);
    loadtestModels(testModels,fileaddress);
//    calcFeatureMatric(trainModels,testModels);
    calcFeatureMatricRankK(trainModels,testModels);
    OFCorrespondece.close();
    OFTrainData.close();
    OFTestData.close();
    //    cout<< " darsade tashkhis "<<correctDetect<<" tedade kol "<<allDetectionNum<<endl;
}


upperbodycore_msgs::Skeleton extract_feature (const std::string &filename)
{
    upperbodycore_msgs::Skeleton msg;
    pcl::console::TicToc tt;
    tt.tic ();
    ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
        stringstream s;
        s<<filename.c_str()<<"  dosen't exist";
        cout<<filename.c_str()<<" doesn't exist "<<endl;
        return msg;
    }
    std::string line;
    int i = 3;
    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.empty ())
        {
            continue;
        }
        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
        geometry_msgs::Vector3 position;
        //        cout<<i<<endl;
        for(int i = 3;i<strs.size(); ++i)
        {
            position.x = boost::lexical_cast<double>(strs.at(i));
            position.y = boost::lexical_cast<double>(strs.at(++i));
            position.z = boost::lexical_cast<double>(strs.at(++i));
            ++i;//confidence
            msg.position.push_back(position);
        }
    }
    fs.close ();
}
bool loadTrainModels (std::vector<features> &models, const std::string filename)
{
    for(int i = 1; i<80 ; ++i)
    {
        string floor_data_file = filename;
        if(i>9)
            floor_data_file.append(to_string(i)+"/floor/floor.txt");
        else
            floor_data_file.append("0"+to_string(i)+"/floor/floor.txt");
        //        cout<<"i= "<<i<<" ";
        ifstream fs;
        fs.open (floor_data_file.c_str ());
        string line;
        getline (fs, line);
        if (line.empty ())
        {
            continue;
        }
        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
        double floorCoefficient[4];
        for(int q = 0;q<strs.size(); ++q)
        {
            floorCoefficient[q] = boost::lexical_cast<double>(strs.at(q));
            //            cout<<"  q"<<q<<" "<<floorCoefficient[q];

        }
        //        cout<<endl;
        fs.close();
        gClient->set_floor(floorCoefficient[0],floorCoefficient[1],floorCoefficient[2],floorCoefficient[3]);
        features modelFeature;
        for (int f= 1; f<featuresCount ;++f)
        {
            modelFeature.d[f] = 0;
        }
        for(int j=1; j<6; ++j)
        {
            string path = filename ;
            if(i<10)
            {
                path.append("0"+to_string(i)+"/skeleton/0"+to_string(j)+".txt");
            }
            else
            {
                path.append(to_string(i)+"/skeleton/0"+to_string(j)+".txt");
            }
            modelFeature.model = i;
            upperbodycore_msgs::Skeleton Skeletonfeature = extract_feature(path);//feature calculation
            gClient->SkeletonData_CB(Skeletonfeature);
            //            cout<<endl;
            double *calcFeature = gClient->get_features_skeleton_article();
            for (int i= 1; i<featuresCount ;++i)
            {
                modelFeature.d[i] += calcFeature[i];
            }
        }
        OFTrainData<<" model: "<<to_string(i);
        for (int i= 1; i<featuresCount ;++i)// train average skeleton data of frames
        {
            modelFeature.d[i] /= 5;// 5 frame amuzeshi darim baraye har fard
            OFTrainData<<", d"<<i<<" = "<<modelFeature.d[i];
        }
        OFTrainData<<"\n";
        models.push_back(modelFeature);
        //        cout<<models.size()<<" models size "<<endl;
    }

    double d[models.size()];
    for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
    {
        for(int i =0; i<models.size(); ++i)
        {
            d[i] = models.at(i).d[j];
        }
        //        cout<<models.size()<<" suize "<<endl;
        double *ave_var_temp = gClient->calc_variance_average(d,models.size());
        ave_var[j][0]=ave_var_temp[0];
        ave_var[j][1]=ave_var_temp[1];
        //        cout<<ave_var[j][0]<<" miangin "<<ave_var[j][1]<<" variance "<<endl;
    }

    for(int i =0; i<models.size(); ++i)
    {
        for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
        {
            models.at(i).d[j] = (models.at(i).d[j]-ave_var[j][0])/ave_var[j][1];
        }
    }

}
void loadtestModels (std::vector<features> &models, const std::string filename)
{

    for(int i = 1; i<80 ; ++i)
    {
        string floor_data_file = filename;
        if(i>9)
            floor_data_file.append(to_string(i)+"/floor/floor.txt");
        else
            floor_data_file.append("0"+to_string(i)+"/floor/floor.txt");
        //        cout<<"i= "<<i<<" ";
        ifstream fs;
        fs.open (floor_data_file.c_str ());
        string line;
        //        cout<<floor_data_file<<" floor data path "<<endl;
        getline (fs, line);
        if (line.empty ())
        {
            continue;
        }
        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
        double floorCoefficient[4];
        for(int q = 0;q<strs.size(); ++q)
        {
            floorCoefficient[q] = boost::lexical_cast<double>(strs.at(q));
        }
        fs.close();
        gClient->set_floor(floorCoefficient[0],floorCoefficient[1],floorCoefficient[2],floorCoefficient[3]);
        features modelFeature;
        for (int f= 1; f<featuresCount ;++f)
        {
            modelFeature.d[f] = 0;
        }
        for(int j=1; j<6; ++j)
        {
            string path = filename ;
            if(i<10)
            {
                path.append("0"+to_string(i)+"/skeleton/0"+to_string(j)+".txt");
            }
            else
            {
                path.append(to_string(i)+"/skeleton/0"+to_string(j)+".txt");
            }
            modelFeature.model = i;
            upperbodycore_msgs::Skeleton Skeletonfeature = extract_feature(path);//feature calculation
            gClient->SkeletonData_CB(Skeletonfeature);
            //            cout<<endl;
            double *calcFeature = gClient->get_features_skeleton_article();
            for (int i= 1; i<featuresCount ;++i)
            {
                modelFeature.d[i] += calcFeature[i];
            }
        }
        OFTestData<<" model: "<<to_string(i);
        for (int i= 1; i<featuresCount ;++i)// train average skeleton data of frames
        {
            modelFeature.d[i] /= 5;// 5 frame amuzeshi darim baraye har fard
            OFTestData<<", d"<<i<<" = "<<modelFeature.d[i];
        }
        OFTestData<<"\n";
        models.push_back(modelFeature);
    }
    double d[models.size()];
    for(int i =0; i<models.size(); ++i)
    {
        for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
        {
            models.at(i).d[j] = (models.at(i).d[j]-ave_var[j][0])/ave_var[j][1];
        }
    }
}
void calcFeatureMatric(std::vector<features> &trainModels,std::vector<features> &testModels)
{
    double featureMatrix[featuresCount][trainModels.size()][testModels.size()];

    int minTrainIndex = -1;
    double minValue = 1000;

    double percentage[featuresCount];
    for(int k=1; k<featuresCount; ++k)//chand ta feature darim barrasi shavad
    {
        percentage[k] = 0;
        for(int j =0; j<testModels.size(); ++j)
        {
            for(int i =0; i<trainModels.size(); ++i)
            {
                featureMatrix[k][i][j] = abs(trainModels.at(i).d[k]- testModels.at(j).d[k]);
                if(featureMatrix[k][i][j]<minValue)
                {
                    minValue =featureMatrix[k][i][j];
                    minTrainIndex = i;
                }
            }
            if(minTrainIndex==j)
            {
                ++percentage[k];
            }
            minValue = 1000;
            minTrainIndex = -1;
        }
        cout<<" tedade dorost "<<percentage[k]<<" "<<" test models size "<<testModels.size()<<" ";
        percentage[k] = (percentage[k]/testModels.size())*100;
        cout<<percentage[k]<<" darsad tashkhis be ezaye vijegie k "<<k<<endl;

    }

}

void calcFeatureMatricRankK(std::vector<features> &trainModels,std::vector<features> &testModels)
{
    double featureMatrix[featuresCount][trainModels.size()][testModels.size()];
    const int rank_k = 10;
    int minTrainIndex[rank_k] = {-1};
    double minValue[rank_k] = {1000};

    double percentage[featuresCount];
    for(int f=1; f<featuresCount; ++f)//chand ta feature darim barrasi shavad
    {
        percentage[f] = 0;
        for(int j =0; j<testModels.size(); ++j)
        {
            for(int i =0; i<trainModels.size(); ++i)
            {
                featureMatrix[f][i][j] = abs(trainModels.at(i).d[f]- testModels.at(j).d[f]);
                if(featureMatrix[f][i][j]<minValue[rank_k-1])
                {
                    int counter = rank_k-1;
                    minValue[rank_k-1] = featureMatrix[f][i][j];
                    minTrainIndex[rank_k-1] = i;
                    while((counter>0)&&(minValue[counter]<minValue[counter-1]))
                    {
//                        cout<<" in while "<<endl;
                        double tempMinValue = minValue[counter];
                        minValue[counter] = minValue[counter-1];
                        minValue[counter-1] = tempMinValue;
                        int tempMinTrainIndex = minTrainIndex[counter];
                        minTrainIndex[counter] = minTrainIndex[counter-1];
                        minTrainIndex[counter-1] = tempMinTrainIndex;
                        --counter;
                    }
                }
            }
            for(int q = 0; q<rank_k; ++q)
            {
//                cout<<" q "<<q<<endl;
                if(minTrainIndex[q]==j)
                {
                    ++percentage[f];
                }
                minValue[q] = 1000;
                minTrainIndex[q] = -1;
            }
        }
        cout<<" tedade dorost "<<percentage[f]<<" "<<" test models size "<<testModels.size()<<" ";
        percentage[f] = (percentage[f]/testModels.size())*100;
        cout<<percentage[f]<<" darsad tashkhis be ezaye vijegie f "<<f<<endl;
    }
}

