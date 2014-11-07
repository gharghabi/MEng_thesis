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
FindMe findMe[4];

struct features
{
    double d[13];
    string model;
    int aveNum;//tedade vijegi hayi ke az an ha miangin gerefte shode ast
};
int featuresCount = 8;
double ave_var[13][2];
calc_features *gClient;
readDataSet * readDataSetClient;
std::vector<features> testModels,trainModels;
bool loadTrainModels(std::vector<features> &trainModels);
void loadtestModels ();
void init();
std::vector<int> correspondingIndex(double *calcFeatures,std::vector<features> &trainModels);
vector<string> all_files[dataNum];
string dataLocation;

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
    dataLocation = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data";
    string fileaddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/walking2/";
    string trainFileAddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/collaborative/";
    string Trainfeatures = ros::package::getPath("mythesis_body")+"/src/activityFirstFeatures.txt";
    string Testfeatures = ros::package::getPath("mythesis_body")+"/src/activityAllFeatures.txt";
    //    string correspondenseResult = ros::package::getPath("mythesis_body")+"/src/correspondenceFeatures.txt";
    //    OFCorrespondece.open (correspondenseResult.c_str ());

    //    "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512172825/RGB_3.png"
    //    /media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data4/0512150222/RGB_1.png
    //                /media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data3/0511121410/RGB_1.png
    //            /media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data2/0510160858/RGB_17.png
    //    camImage = imread("/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data2/0510160858/RGB_17.png");
    //    FaceDetect(5,5);
    //    imshow("sala,",camImage);
    readDataSetClient = new readDataSet("/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/");
    OFTrainData.open (Trainfeatures.c_str());
    OFTestData.open(Testfeatures.c_str());

    //     for(int i = 0; i<4; ++i)
    //     {
    //        for(int j=0; j<all_files[i].size(); ++j)
    //         cout<<i<<" file "<<all_files[i].at(j)<<endl;
    //     }
    loadTrainModels(trainModels);//??????????dade haye amuzeshi eshtebah mibashad
    //    loadtestModels();
    //    OFCorrespondece.close();
    OFTrainData.close();
    OFTestData.close();
}
//dadahaye file skeleton avvali shomare frame

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
        cout<<" frame num "<<frameNum<<endl;
        for(int j = 1; j<(frameNum+1); ++j)
        {
            modelFeature.model = i+1;
            readDataSetClient->set_frameNum(j);
            upperbodycore_msgs::Skeleton Skeletonfeature = readDataSetClient->get_skeletonData();//feature calculation
            ROS_INFO(" frame num, %d ",j);
            cv::Mat RGBImage = readDataSetClient->get_rgbImage();

            if((RGBImage.cols>0))
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
        cout<<" 103 "<<endl;
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
// baraye data1 data2 data3
//bool FaceDetect(int a, int b)
//{
//    std::vector<cv::Rect> faces;
////    "/home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_default.xml";
////    /home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_alt2.xml
//              String face_cascade_name = "/home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_alt2.xml";
//int flags_face= CV_HAAR_DO_ROUGH_SEARCH && CV_HAAR_FEATURE_MAX && CV_HAAR_DO_CANNY_PRUNING;//|CV_HAAR_FIND_BIGGEST_OBJECT
//              cv::CascadeClassifier face_cascade;
//              face_cascade = cv::CascadeClassifier(face_cascade_name);
//              Mat gray;
//              cvtColor(camImage,gray,CV_BGR2GRAY);
//              if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); };
//                 face_cascade.detectMultiScale( gray, faces, 1.2, 2, flags_face|CASCADE_SCALE_IMAGE, Size(0, 0) );//khub ba yek ghalat gray, faces, 1.2, 10, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) )
//                for( size_t i = 0; i < faces.size(); i++ )
//                    {
//                Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
//                  ellipse( camImage, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
//              }
//              imshow("salam",camImage);
//              waitKey(0);

//            if (faces.size()!=0)
//            {
//                return true;
//            }
//            else
//                return false;

//}




bool FaceDetect(int a, int b)
{
    std::vector<cv::Rect> faces;
    //    "/home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_default.xml";
    //    /home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_alt2.xml
    String face_cascade_name = "/home/shaghayegh/catkin_ws/src/thesis_skeleton_face/haarcascade_frontalface_alt2.xml";
    int flags_face= CV_HAAR_DO_ROUGH_SEARCH && CV_HAAR_FEATURE_MAX && CV_HAAR_DO_CANNY_PRUNING;//|CV_HAAR_FIND_BIGGEST_OBJECT
    cv::CascadeClassifier face_cascade;
    face_cascade = cv::CascadeClassifier(face_cascade_name);
    Mat gray;
    cvtColor(camImage,gray,CV_BGR2GRAY);
    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); };
    face_cascade.detectMultiScale( gray, faces, 1.1, 1, flags_face|CASCADE_SCALE_IMAGE, Size(0, 0) );//khub ba yek ghalat gray, faces, 1.2, 10, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) )
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( camImage, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
    }
    imshow("salam",camImage);
    waitKey(0);

    if (faces.size()!=0)
    {
        return true;
    }
    else
        return false;
}
void loadtestModels ()
{
    int percentage[featuresCount];
    for(int w=0; w<featuresCount; ++w)
    {
        percentage[w] = 0;
    }
    int allFrameNum = 0;
    for(int i = 0; i<dataNum; ++i)
    {
        int frameNum = readDataSetClient->set_parameters((i+1),"still");
        //        for (int f= 1; f<featuresCount ;++f)
        //        {
        //            modelFeature.d[f] = 0; //?mage meghdar dehi avvalie sefr nemishe?
        //        }
        cout<<" frame num "<<frameNum<<endl;
        Rect result;
        for(int j = 1; j<(frameNum+1); ++j)
        {
            features modelFeature;
            //                for (int f= 1; f<featuresCount ;++f)
            //                {
            //                    modelFeature.d[f] = 0;
            //                }
            modelFeature.model = i+1;
            readDataSetClient->set_frameNum(j);
            upperbodycore_msgs::Skeleton Skeletonfeature = readDataSetClient->get_skeletonData();//feature calculation
            ROS_INFO(" frame num, %d ",j);
            cv::Mat RGBImage = readDataSetClient->get_rgbImage();
            (&findMe[i])->updateImage(RGBImage);
            (&findMe[i])->doWork();
            float conf = (&findMe[i])->getRect(result);
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
                OFTestData<<" correponding = "<<corresponding.at(q)<<"\n";
                if(corresponding.at(q) == i)
                {
                    percentage[q]++;
                }
            }
        }
        OFTestData<<" model: "<<boost::to_string(i);
        OFTestData<<"\n";
    }
    for(int q = 0; q<featuresCount; ++q)
    {
        cout<<" tedade dprost "<<percentage[q]<<endl;
        cout<<allFrameNum<<endl;

        float darsad = (float(percentage[q])/float(allFrameNum))*100;
        cout<<q<<" feature "<<darsad<<" darsad" <<endl;
    }
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

