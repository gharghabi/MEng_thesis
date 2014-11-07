#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <ros/ros.h>
//#include "calc_features.h"
#include <pcl/console/time.h>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include "Config.h"
#include "FindMe.h"
#include "readDataSet.h"
using namespace std;
using tld::Config;
using tld::Settings;
using namespace cv;

std::ofstream OFTrainData,OFTestData;
std::ofstream OFCorrespondece;
const int dataNum = 4;
const int activitesNum = 17;
FindMe findMe[dataNum];
//
string activityName[activitesNum]={"still","talking on the phone","writing on whiteboard","drinking water","rinsing mouth with water","brushing teeth",
                                   "wearing contact lenses1","wearing contact lenses2","talking on couch","relaxing on couch",
                                   "cooking (chopping)",
                                   "cooking (stirring)","opening pill container1","opening pill container2","opening pill container3",
                                   "working on computer","random"};

struct features
{
    double d[16];
    string model;
    int aveNum;//tedade vijegi hayi ke az an ha miangin gerefte shode ast
};
int featuresCount = 16;
double ave_var[16][2];
calc_features *gClient;
readDataSet * readDataSetClient;
std::vector<features> testModels,trainModels;
bool loadTrainModels(std::vector<features> &trainModels);
void loadtestModels ();
void init();
std::vector<int> correspondingIndex(double *calcFeatures,std::vector<features> &trainModels);
vector<string> all_files[dataNum];

int argc;
char **argv;
cv::Mat camImage;
bool FaceDetect(int a, int b);
int main(int mainargc,char **mainargv) {

    ros::init(mainargc, mainargv, "skeletonTestOnActivityDataset");
    argc = mainargc;
    argv = mainargv;
    gClient = new calc_features();
    init();

    cout<<" git  "<<endl;


    string Trainfeatures = ros::package::getPath("thesis_skeleton_face")+"/src/activityFirstFeatures.txt";
    string Testfeatures = ros::package::getPath("thesis_skeleton_face")+"/src/activityAllFeatures.txt";
    readDataSetClient = new readDataSet("/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/");
    OFTrainData.open (Trainfeatures.c_str());
    OFTestData.open(Testfeatures.c_str());
    loadTrainModels(trainModels);
    loadtestModels();
    OFTrainData.close();
    OFTestData.close();
}

bool loadTrainModels (std::vector<features> &models)
{
    features modelFeature;
    for(int i = 0; i<dataNum; ++i)
    {
        int frameNum = readDataSetClient->set_parameters((i+1),"still");
        //        for (int f= 1; f<featuresCount ;++f)
        //        {
        //            modelFeature.d[f] = 0; //?mage meghdar dehi avvalie sefr nemishe?
        //        }
//        cout<<" frame num "<<frameNum<<endl;
        for(int j = 1; j<(frameNum+1); ++j)
        {
            modelFeature.model = i+1;
            readDataSetClient->set_frameNum(j);
            upperbodycore_msgs::Skeleton Skeletonfeature = readDataSetClient->get_skeletonData();//feature calculation
            ROS_INFO(" frame num, %d ",j);
            cv::Mat RGBImage = readDataSetClient->get_rgbImage();

            if((RGBImage.cols>0)&&(j==1))
            {
                (&findMe[i])->updateImage(RGBImage);
                (&findMe[i])->detectFace();
                (&findMe[i])->doWork();
            }
            else
            {
                (&findMe[i])->updateImage(RGBImage);
                (&findMe[i])->doWork();
            }

            gClient->SkeletonData_CB(Skeletonfeature);
            double *calcFeature = gClient->get_features_skeleton_my();
            for (int k= 1; k<featuresCount ;++k)
            {
                modelFeature.d[k] += calcFeature[k];
            }
        }
        cout<<" test "<<endl;
        OFTrainData<<" model: "<<boost::to_string(i+1);
        for (int k= 1; k<featuresCount ;++k)// train average skeleton data of frames
        {
            modelFeature.d[k] /= frameNum;// 5 frame amuzeshi darim baraye har fard
            OFTrainData<<", d"<<k<<" = "<<modelFeature.d[k];
        }
        OFTrainData<<"\n";
        models.push_back(modelFeature);
    }
    double d[models.size()];
    for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
    {
        ROS_INFO(" fetures %d",j);
        for(int i =0; i<models.size(); ++i)
        {
            d[i] = models.at(i).d[j];
        }
        double *ave_var_temp = gClient->calc_variance_average(d,models.size());
        ave_var[j][0]=ave_var_temp[0];
        ave_var[j][1]=ave_var_temp[1];
    }
    for(int i =0; i<models.size(); ++i)
    {
        for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
        {
            models.at(i).d[j] = (models.at(i).d[j]-ave_var[j][0])/ave_var[j][1];
            cout<<" person num "<<i<<" vijegie jom "<<j<<" "<<models.at(i).d[j]<<endl;
        }
    }
}

void loadtestModels ()
{
    int percentage[featuresCount+1];//akharie baraye find meie
//    for(int l = 0; l<activitesNum; ++l)
//    {
        int allFrameNum = 0;
        for(int w=0; w<featuresCount; ++w)
        {
            percentage[w] = 0;
        }
        for(int i = 0; i<dataNum; ++i)
        {
            int frameNum = readDataSetClient->set_parameters((i+1),"talking on the phone");//activityName[l]
            Rect result;
            for(int j = 1; j<(frameNum+1); ++j)
            {
                ++allFrameNum;
                features modelFeature;
                modelFeature.model = i+1;
                readDataSetClient->set_frameNum(j);
                upperbodycore_msgs::Skeleton Skeletonfeature = readDataSetClient->get_skeletonData();//feature calculation
                ROS_INFO(" frame num, %d ",j);
                float conf[dataNum];
                float maxConf = -1;
                int maxConfIndex = -1;
                cv::Mat RGBImage = readDataSetClient->get_rgbImage();
                for(int q = 0; q<dataNum; ++q)
                {
                (&findMe[q])->updateImage(RGBImage);
                (&findMe[q])->doWork();
                    conf[q] = (&findMe[q])->getRect(result);
                    if(conf[q]>maxConf)
                    {
                        maxConf = conf[q];
                        maxConfIndex = q;
                    }
                }

                if(maxConfIndex == i)
                {
                    percentage[featuresCount]++;
                }
                gClient->SkeletonData_CB(Skeletonfeature);
                double *calcFeature = gClient->get_features_skeleton_my();
                for(int j=1; j<featuresCount; ++j)//chand ta feature darim barrasi shavad
                {
                    OFTestData<<" f"<<j<<" = "<<calcFeature[j];
                    calcFeature[j] = (calcFeature[j]-ave_var[j][0])/ave_var[j][1];
                }
                std::vector<int> corresponding = correspondingIndex(calcFeature,trainModels);

                //                cout<<"corresponding "<<corresponding.size()<<endl;
                for(int q = 0; q<corresponding.size(); ++q)
                {
                    OFTestData<<" f = "<<q<<" corres "<<corresponding.at(q);
                    if(corresponding.at(q) == i)
                    {
                        percentage[q]++;
                    }
                }
                OFTestData<<"\n";
            }
            cout<<i<<" fard "<<endl;
            OFTestData<<" model: "<<boost::to_string(i);
            OFTestData<<"\n";
        }
        OFTestData<<" recogntion percentae activitye : "<<"talking on the phone"<<"\n";//activityName[l]
        for(int q = 0; q<featuresCount+1; ++q)
        {
            cout<<" tedade dprost "<<percentage[q]<<endl;
            cout<<allFrameNum<<endl;

            float darsad = (float(percentage[q])/float(allFrameNum))*100;
            cout<<q<<" feature "<<darsad<<" darsad" <<endl;
            OFTestData<<" all frame num : "<<allFrameNum<<" number if correct detection: "<<percentage[q]<<" feature: "<<q<<" percentage "<<darsad<<"\n";

        }
//    }
}
std::vector<int> correspondingIndex(double *calcFeature,std::vector<features> &trainModels)
{
    std::vector<int> corresponding;
    int minTrainIndex = -1;
    double minValue = 1000;
    //    cout<<trainModels.size()<<" size "<<endl;
    for(int k=1; k<featuresCount; ++k)//chand ta feature darim barrasi shavad
    {
        for(int i =0; i<trainModels.size(); ++i)
        {
            double temp = abs(trainModels.at(i).d[k]- calcFeature[k]);
            if(temp<minValue)
            {
                minValue = temp;
                minTrainIndex = i;
            }
        }
        corresponding.push_back(minTrainIndex);
        //        cout<<corresponing[k]<<" shabahat "<<endl;

        minValue = 1000;
        minTrainIndex = -1;
    }
    return corresponding;
    //    cout<<" 260 "<<endl;
}

void init() {

    Config config;
    if (config.init(argc, argv) == PROGRAM_EXIT) {
        exit(EXIT_FAILURE);
    }
    config.configure((&findMe[0]));
    config.configure((&findMe[1]));
    config.configure((&findMe[2]));
    config.configure((&findMe[3]));
    ROS_INFO("TLD configured.");

}

