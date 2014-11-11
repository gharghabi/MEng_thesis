#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <ros/ros.h>
// // OpenMP includes.
#include <omp.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
// C++ includes.
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <XnCppWrapper.h>
#include <XnLog.h>

// //includes de mis clases

#define CHECK_RC(rc, what, flag)\
    flag= 1;\
    if (rc != XN_STATUS_OK)\
{\
    cout << what << "failed :"<< xnGetStatusString(rc);\
    flag = 0;\
    }\
    cout << what << "succeed = " << flag;

using namespace cv;
using namespace xn;
using namespace std;

String DepthFile,RGBFile;
xn::Context context;
XnStatus nRetVal;
xn::Player Player_man;
//Generators
bool context_init;
xn::DepthGenerator depth;   bool depth_init;
xn::ImageGenerator color;   bool color_init;
xn::UserGenerator user;     bool user_init;
xn::Recorder recorder;
bool user_cloud_populated;
// Data types
xn::DepthMetaData depthMD;  //Deth Meta data
xn::ImageMetaData imageMD;  //Image Meta Data
xn::SceneMetaData sceneMD;  //Scene meta data - It will be used to segment the USER
const XnRGB24Pixel* pImageMap;    //Pointer to color map
const XnDepthPixel* pDepthMap;    //Pointer to Depth map
bool recorder_init;
bool recorder_file,recorder_depth, recorder_color;
bool startgeneration_init;
bool aligment;
MockDepthGenerator mockg_Depth;
MockImageGenerator mockg_Image;

xn::MockRawGenerator rawGenerator; bool rawGenerator_init;

int main( int argc,  char** argv )
{
    ros::init(argc, argv, "openni_tracker");
    String directorio = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512164529/";
        nRetVal = XN_STATUS_OK;
    depth_init =0;
    color_init =0;
    context_init =0;
    recorder_init =0;

    context.Release();//@@
    nRetVal = context.InitFromXmlFile("/home/shaghayegh/backup/SamplesConfig.xml");
//    CHECK_RC(nRetVal, "InitFromXml");
//    nRetVal = context.Init();//@@?initializ kardane kinect
//    CHECK_RC(nRetVal, "Intialization",context_init);//@@

//        nRetVal = context.OpenFileRecording( "/home/shaghayegh/backup/up.oni", Player_man);//@@?baz kardane fili ke tush etelaat zakhire shode
//    CHECK_RC(nRetVal, "Recording opening",context_init);//@@
    // Intialize Depth
    nRetVal = depth.Create(context);
    CHECK_RC(nRetVal, "Depth Initialization",depth_init);


    // Intialize Color
    nRetVal = color.Create(context);
    CHECK_RC(nRetVal, "Color Initialization",color_init);

    // Intialize User
    nRetVal = user.Create(context);
    CHECK_RC(nRetVal, "Color Initialization",user_init);



    if(depth.IsCapabilitySupported("AlternativeViewPoint"))
    {
        nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(color);
        CHECK_RC(nRetVal,"Depth and Color Aligment",aligment);
    }
    else
    {
        cout<< "Depth and Color aligment is  **** NOT **** possible";
    }

    // Creating Production Tree
    context.CreateAnyProductionTree(XN_NODE_TYPE_RECORDER,NULL,recorder)   ;
    CHECK_RC(nRetVal, "Create recorder",recorder_init);


    nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, "/home/shaghayegh/shcode.oni" );
    CHECK_RC(nRetVal, "Setting up recorder file ",recorder_file);

    nRetVal = recorder.AddNodeToRecording(depth, XN_CODEC_16Z_EMB_TABLES);
    CHECK_RC(nRetVal, "Adding Node to Recorder - Depth ",recorder_depth);

    nRetVal = recorder.AddNodeToRecording(color);
    CHECK_RC(nRetVal, "Adding Node to Recorder - Color ",recorder_color);

    nRetVal = context.StartGeneratingAll();
//    CHECK_RC(nRetVal, "Kinect Final configuration ",startgeneration_init);
    // boost::thread Update_Process(&UpdateContext);
    // ros::Rate r(30);
    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);
    int frames = 0;
    while (ros::ok()) {
              nRetVal = context.WaitAnyUpdateAll(); // es el que va mejor!!!!!
              depth.GetMetaData(depthMD);
              pDepthMap = depthMD.Data();

              color.GetMetaData(imageMD);
              pImageMap = color.GetRGB24ImageMap();

            recorder.Record();
            ros::spinOnce();
    }
    if(depth_init) depth.Release();
    if(color_init) color.Release();
    if(context_init) context.Release();
    if(recorder_init) recorder.Release();

    context.Shutdown();
    return 0;
}
