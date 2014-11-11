#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>
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

// OpenCV includes.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

// OpenMP includes.
// #include <omp.h>

// C++ includes.
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "/home/shaghayegh/backup/SamplesConfig.xml"
//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what) \
    if (rc != XN_STATUS_OK) \
{ \
    printf("%s failed: %s\n", what, xnGetStatusString(rc)); \
    return rc; \
    }
//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
using namespace xn;
using namespace cv;
using namespace std;

void transformDepthMD(DepthMetaData& depthMD)
{
    DepthMap& depthMap = depthMD.WritableDepthMap();
    for (XnUInt32 y = 0; y < depthMap.YRes(); y++)
    {
        for (XnUInt32 x = 0; x < depthMap.XRes(); x++)
        {
            //Punch vertical cut lines in the depth image
           if ((x % 2) == 0)
           {

                depthMap(x,y) = 0;
           }
        }
    }
}
void transformImageMD(Mat FrameImage,ImageMetaData& imageMD)
{
    RGB24Map& imageMap = imageMD.WritableRGB24Map();
    for (XnUInt32 y = 0; y < imageMD.YRes(); y++)
    {
        for (XnUInt32 x = 0; x <imageMD.XRes(); x++)
        {
            cout<<" x "<<x<<" y "<<y<<endl;
            XnRGB24Pixel imagePixel;
            imagePixel.nBlue=FrameImage.at<Vec3b>(y,x)[0];
            imagePixel.nGreen=FrameImage.at<Vec3b>(y,x)[1];
            imagePixel.nRed=FrameImage.at<Vec3b>(y,x)[2];
            imageMap(x,y) = imagePixel;
                        cout<<" 76 "<<endl;
        }
    }

}

int main(int argc, char* argv[])
{
    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = xnLogInitFromXmlFile(SAMPLE_XML_PATH);
    if (nRetVal != XN_STATUS_OK)
    {
        printf("Log couldn't be opened: %s. Running without log", xnGetStatusString(nRetVal));
    }
    if (argc < 3)
    {
        printf("usage: %s <inputFile> <outputFile>\n", argv[0]);
        return -1;
    }
    const char* strInputFile = argv[1];
    const char* strOutputFile = argv[2];
    Context context;
    nRetVal = context.Init();
    CHECK_RC(nRetVal, "Init");
    // open input file
    Player player;
    nRetVal = context.OpenFileRecording("/home/shaghayegh/backup/up.oni", player);
    CHECK_RC(nRetVal, "Open input file");
    // Get depth node from recording
    DepthGenerator depth;
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
    CHECK_RC(nRetVal, "Find depth generator");
    // Create mock node based on depth node from recording
    MockDepthGenerator mockDepth;
    nRetVal = mockDepth.CreateBasedOn(depth);
    CHECK_RC(nRetVal, "Create mock depth node");


    ImageGenerator image;
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
    CHECK_RC(nRetVal, "Find depth generator");
    // Create mock node based on depth node from recording
    // MockImageGenerator mockImage;
    // nRetVal = mockImage.CreateBasedOn(image);
    CHECK_RC(nRetVal, "Create mock depth node");
    // create recorder
    Recorder recorder;
    nRetVal = recorder.Create(context);
    CHECK_RC(nRetVal, "Create recorder");
    nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, "/home/shaghayegh/up.oni");
    CHECK_RC(nRetVal, "Set recorder destination file");
    // add depth node to recorder
    nRetVal = recorder.AddNodeToRecording(mockDepth);
    CHECK_RC(nRetVal, "Add node to recording");
    // nRetVal = recorder.AddNodeToRecording(mockImage);
    // CHECK_RC(nRetVal, "Add node to recording");

    nRetVal = player.SetRepeat(FALSE);
    XN_IS_STATUS_OK(nRetVal);
    XnUInt32 nNumFrames = 0;
    nRetVal = player.GetNumFrames(image.GetName(), nNumFrames);
    CHECK_RC(nRetVal, "Get player number of frames");
    DepthMetaData depthMD;
    // ImageMetaData imageMD;
    int frameNum = 0;
    String path = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512164529";
    while ((nRetVal = depth.WaitAndUpdateData()) != XN_STATUS_EOF)
    {
        ++frameNum;
        CHECK_RC(nRetVal, "Read next frame");
        // Get depth meta data
        depth.GetMetaData(depthMD);
        // image.GetMetaData(imageMD);

        //-----------------------------------------------//
        // Transform depth! This is the interesting part //
        //-----------------------------------------------//
        /* Enable the depth data to be modified. This is done implicitly by depthMD.WritableDepthMap(),
but we're calling it just to be clear. */
        nRetVal = depthMD.MakeDataWritable();
        CHECK_RC(nRetVal, "Make depth data writable");

        // nRetVal = imageMD.MakeDataWritable();
        // CHECK_RC(nRetVal, "Make depth data writable");

        // String ficheroActualRGB;
        // ficheroActualRGB = path  +"RGB_" + boost::to_string(frameNum) + ".png";
        // String ficheroActualDepth = path +"Depth_"+ boost::to_string(frameNum) + ".png";

        // Mat matFrameImage = imread(ficheroActualRGB, 1);
        // resize(matFrameImage, matFrameImage, Size(640, 480), 0, 0, INTER_CUBIC);
        // Mat matFrameDepth = imread(ficheroActualDepth,1);
        // resize(matFrameDepth, matFrameDepth, Size(640, 480), 0, 0, INTER_CUBIC);

        transformDepthMD(depthMD);
        // transformImageMD(matFrameImage,imageMD);
//         Pass the transformed data to the mock depth generator
        nRetVal = mockDepth.SetData(depthMD);
        CHECK_RC(nRetVal, "Set mock node new data");

        // nRetVal = mockImage.SetData(imageMD);
        // CHECK_RC(nRetVal, "Set mock node new data");

        /* We need to call recorder.Record explicitly because we're not using WaitAndUpdateAll(). */
        nRetVal = recorder.Record();
        CHECK_RC(nRetVal, "Record");
        printf("Recorded: frame %u out of %u\r", depthMD.FrameID(), nNumFrames);
    }
    printf("\n");
    return 0;
}
