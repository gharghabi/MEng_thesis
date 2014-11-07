#include "calc_features.h"
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

struct frameData{
    cv::Mat RGBImage;
    cv::Mat depthImage;
    upperbodycore_msgs::Skeleton skeletonData;
    std::string skeletonDataLine;
};


class readDataSet{

private:
    static int const dataNum = 4;
    int frameNum;
    int personNum;
    cv::Mat RGBImage;
    cv::Mat depthImage;
    frameData allData;
    std::string action;
    std::string dataLocation;
    std::string skeletonDataLine;
    upperbodycore_msgs::Skeleton skeletonData;
    std::vector<std::string> skeletonFilesLine;
    std::map<std::string, std::string> all_files[dataNum];

    void readDataActMap();
    void readSkeletonLines();
    void get_data();
    upperbodycore_msgs::Skeleton extractData (const std::string line);

public:

    readDataSet(std::string baseAddress);
    ~readDataSet();
    cv::Mat get_rgbImage();
    cv::Mat get_depthImage();
    frameData get_frameData();
    std::string get_skeletonFileData();
    void set_frameNum(int frameNumber);
    int set_parameters(int dataNum, std::string actionName);
    upperbodycore_msgs::Skeleton get_skeletonData();


};
