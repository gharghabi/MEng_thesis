#include "readDataSet.h"

using namespace std;
readDataSet::readDataSet(string address)
{
    frameNum = 0;
    dataLocation = address;
    readDataActMap();
}
readDataSet::~readDataSet()
{
    depthImage.release();
    RGBImage.release();
}
void readDataSet::get_data()
{
    string dataAddress = all_files[personNum-1][action];
    string depthImageAddress = dataAddress +"/Depth_"+ boost::to_string(frameNum) +".png";
    string RGBImageAddress = dataAddress +"/RGB_"+ boost::to_string(frameNum) +".png";
//    cout<<RGBImageAddress<<" image address "<<endl;
    RGBImage = cv::imread(RGBImageAddress);
    depthImage = cv::imread(depthImageAddress);
    skeletonDataLine = skeletonFilesLine.at(frameNum-1);
    skeletonData = extractData(skeletonFilesLine.at(frameNum-1));
}
void readDataSet::readSkeletonLines()
{
    skeletonFilesLine.clear();
    string dataAddress = all_files[personNum-1][action];
    string skeletonAddress =dataAddress +".txt";
    ifstream fs;
    fs.open (skeletonAddress.c_str ());
    string line;
    getline (fs, line);
    while(line!="END")
    {
        if (line.empty ())
            continue;
        skeletonFilesLine.push_back(line);
        getline (fs, line);
    }
//    cout<<" files line size "<<skeletonFilesLine.size()<<endl;
    fs.close();
}
int readDataSet::set_parameters(int dataNum, string actionName)
{
    action = actionName;
    personNum = dataNum;
    readSkeletonLines();
    return skeletonFilesLine.size();
}
void readDataSet::set_frameNum(int frameNumber)
{
    frameNum = frameNumber;
    get_data();
}
cv::Mat readDataSet::get_rgbImage()
{
    return RGBImage;
}
cv::Mat readDataSet::get_depthImage()
{
    return depthImage;
}
upperbodycore_msgs::Skeleton readDataSet::get_skeletonData()
{
    return skeletonData;
}
string readDataSet::get_skeletonFileData()
{
    return skeletonDataLine;
}
frameData readDataSet::get_frameData()
{

    allData.RGBImage = RGBImage;
    allData.depthImage = depthImage;
    allData.skeletonData = skeletonData;
    allData.skeletonDataLine = skeletonDataLine;
    return allData;
}
void readDataSet::readDataActMap() {
    for(int i = 0; i<dataNum; ++i)
    {
        const string mapfile = dataLocation + "data" + boost::to_string(i+1)+ "/activityLabel.txt";
        string baseAddress =  dataLocation + "data" + boost::to_string(i+1)+"/";
//        cout<<mapfile<<endl;
        map<string, string> data_act_map;

        // printf("Opening map of data to activity: \"%s\"\n", (char*)mapfile.c_str());
        ifstream file((char*)mapfile.c_str(), ifstream::in);
        string line;
        int count = 0;
        while(getline(file,line)) {
            // stringstream lineStream(line);
            string element1, element2;
            std::vector<std::string> strs;
            boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
            element1 = strs.at(0);
            if (element1.compare("END") == 0) {
                break;
            }
            element2 = strs.at(1);
            if (element1.length() != 10) {
                ROS_ERROR("Data Act Map file format mismatch..");
            }
            string element3 = baseAddress + element1;
            data_act_map[element2] = element3;
//            cout<<data_act_map[element2]<<" element 1 "<<element1<<" "<<element2<<" element2 "<<endl;
            // cout << "\t" << element1  << " -> \"" << data_act_map[element1] << "\"" << endl;
            count++;
        }
        file.close();
        if(count == 0) {
            ROS_ERROR("File does not exist or is empty!\n");
        }
        all_files[i] =  data_act_map;
    }
}
upperbodycore_msgs::Skeleton readDataSet::extractData (const std::string line)
{
    upperbodycore_msgs::Skeleton msg;
    std::vector<std::string> strs;
    boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation;
    Eigen::Matrix3f rotationMatrix;

    float R1,R2,R3,R4,R5,R6,R7,R8,R9;
    int jointNum = 5;
    int i=0;
    while(i<strs.size()-2)
    {
        if((msg.confidence.size()==5)||(msg.confidence.size()==8)||(msg.confidence.size()==11)||(msg.confidence.size()==14))
        {
            msg.orientation.push_back(orientation);
            msg.position.push_back(position);
            msg.confidence.push_back(1);
        }
        else if(msg.confidence.size()==15)
        {
            position.x = boost::lexical_cast<double>(strs.at(++i));
            position.y = boost::lexical_cast<double>(strs.at(++i));
            position.z = boost::lexical_cast<double>(strs.at(++i));
            float conf = boost::lexical_cast<float>(strs.at(++i));
            msg.confidence.at(jointNum) = conf;
            msg.position.at(jointNum) = position;//check shavad ke dorost kar mikone ya na
            jointNum += 3;
        }
        else
        {
            R1 = boost::lexical_cast<double>(strs.at(++i));   R2 = boost::lexical_cast<double>(strs.at(++i));   R3 = boost::lexical_cast<double>(strs.at(++i));
            R4 = boost::lexical_cast<double>(strs.at(++i));   R5 = boost::lexical_cast<double>(strs.at(++i));   R6 = boost::lexical_cast<double>(strs.at(++i));
            R7 = boost::lexical_cast<double>(strs.at(++i));   R8 = boost::lexical_cast<double>(strs.at(++i));   R9 = boost::lexical_cast<double>(strs.at(++i));
            rotationMatrix<<R1,R2,R3,
                    R4,R5,R6,
                    R7,R8,R9;

            Eigen::Quaternionf q(rotationMatrix);
            orientation.w = q.w();
            orientation.x = q.x();
            orientation.y = q.y();
            orientation.z = q.z();
            ++i;//inja ye ++i dge ham bayad bezaram? baraye conf dovvom
            position.x = boost::lexical_cast<double>(strs.at(++i));
            position.y = boost::lexical_cast<double>(strs.at(++i));
            position.z = boost::lexical_cast<double>(strs.at(++i));
            float conf = boost::lexical_cast<float>(strs.at(++i));
            msg.confidence.push_back(conf);
            msg.orientation.push_back(orientation);
            msg.position.push_back(position);
        }
    }
    return msg;
}
