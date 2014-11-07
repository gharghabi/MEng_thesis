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
const int dataNum = 4;
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
void readDataActMap();
bool loadTrainModels(std::vector<features> &trainModels);
void loadtestModels ();
std::vector<int> correspondingIndex(double *calcFeatures,std::vector<features> &trainModels);
upperbodycore_msgs::Skeleton extract_feature (const std::string filename);
vector<string> all_files[dataNum];
string dataLocation;

int main(int argc, char **argv) {

    ros::init(argc, argv, "skeletonTestOnActivityDataset");

    gClient = new calc_features();
    dataLocation = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data";
    string fileaddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/walking2/";
    string trainFileAddress = "/home/shaghayegh/catkin_ws/src/mythesis_body/dataset/RGBD-ID/data/collaborative/";
    string Trainfeatures = ros::package::getPath("mythesis_body")+"/src/activityFirstFeatures.txt";
    string Testfeatures = ros::package::getPath("mythesis_body")+"/src/activityAllFeatures.txt";
    //    string correspondenseResult = ros::package::getPath("mythesis_body")+"/src/correspondenceFeatures.txt";
    //    OFCorrespondece.open (correspondenseResult.c_str ());
    OFTrainData.open (Trainfeatures.c_str());
    OFTestData.open(Testfeatures.c_str());
    readDataActMap();
    //     for(int i = 0; i<4; ++i)
    //     {
    //        for(int j=0; j<all_files[i].size(); ++j)
    //         cout<<i<<" file "<<all_files[i].at(j)<<endl;
    //     }
    cout<<"commit "<<endl;
    loadTrainModels(trainModels);//??????????dade haye amuzeshi eshtebah mibashad
    loadtestModels();
    //    OFCorrespondece.close();
    OFTrainData.close();
    OFTestData.close();
}
//dadahaye file skeleton avvali shomare frame
void readDataActMap() {
    for(int i = 1; i<(dataNum+1); ++i)
    {
        const string mapfile = dataLocation + to_string(i)+ "/activityLabel.txt";
        string baseAddress =  dataLocation + to_string(i)+"/";
        cout<<mapfile<<endl;
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
            data_act_map[element1] = element2;
            // cout << "\t" << element1  << " -> \"" << data_act_map[element1] << "\"" << endl;
            count++;
        }
        file.close();
        if(count == 0) {
            ROS_ERROR("File does not exist or is empty!\n");
        }
        // printf("\tcount = %d\n\n", count);
        map<string,string>::iterator it = data_act_map.begin();
        while(it != data_act_map.end()) {
            string completeAddress = baseAddress + it->first+".txt";
            all_files[(i-1)].push_back(completeAddress);
            it++;
        }
    }
}

bool loadTrainModels (std::vector<features> &models)
{
    for(int i = 1; i<(dataNum+1) ; ++i)
    {
        int frameNum = 0;
        string skeleton_data = all_files[i-1].at(0);
        cout<<skeleton_data<<" address file "<<endl;
        ifstream fs;
        fs.open (skeleton_data.c_str ());
        string line;
        features modelFeature;
        for (int f= 1; f<featuresCount ;++f)
        {
            modelFeature.d[f] = 0;
        }
        getline (fs, line);
        while(line!="END")
        {
            // cout<<" 125 "<<endl;
            if (line.empty ())
                continue;
            // cout<<" 130 "<<endl;
            modelFeature.model = i;
            // cout<<" 132 "<<endl;

            upperbodycore_msgs::Skeleton Skeletonfeature = extract_feature(line);//feature calculation
            // cout<<" 132 "<<endl;
            gClient->SkeletonData_CB(Skeletonfeature);
            double *calcFeature = gClient->get_features_skeleton_my();

            for (int i= 1; i<featuresCount ;++i)
            {
                modelFeature.d[i] += calcFeature[i];
            }
            // cout<<" frame num "<<frameNum<<endl;
            ++frameNum;
            getline (fs, line);
        }
        fs.close();
        OFTrainData<<" model: "<<to_string(i);
        for (int i= 1; i<featuresCount ;++i)// train average skeleton data of frames
        {
            modelFeature.d[i] /= frameNum;// 5 frame amuzeshi darim baraye har fard
            OFTrainData<<", d"<<i<<" = "<<modelFeature.d[i];
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
void loadtestModels ()
{
    int percentage[featuresCount];
    for(int w=0; w<featuresCount; ++w)
    {
        percentage[w] = 0;
    }
    int allFrameNum = 0;
    for(int i = 1; i<(dataNum+1) ; ++i)
    {
        for(int j = 0; j<all_files[i-1].size(); ++j)
        {
            string floor_data_file = all_files[i-1].at(j);
            ifstream fs;
            fs.open (floor_data_file.c_str());
            string line;
            OFTestData<<floor_data_file.c_str()<<" ";
            //                    cout<<floor_data_file<<" floor data path "<<endl;
            getline (fs, line);
            while(line!="END")
            {
                if (line.empty ())
                    continue;
                std::vector<std::string> strs;
                boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
                features modelFeature;
                for (int f= 1; f<featuresCount ;++f)
                {
                    modelFeature.d[f] = 0;
                }
                modelFeature.model = i;
                upperbodycore_msgs::Skeleton Skeletonfeature = extract_feature(line);//feature calculation
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
                ++allFrameNum;
                getline (fs, line);
            }
            fs.close();
        }
        OFTestData<<" model: "<<to_string(i);
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
upperbodycore_msgs::Skeleton extract_feature (const std::string line)
{
    upperbodycore_msgs::Skeleton msg;
    pcl::console::TicToc tt;
    tt.tic ();
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
        // cout<<" line "<<line<<endl;
        // cout<<"i "<<i<<" size msg "<<msg.confidence.size()<<endl;
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
            // cout<<" x "<<position.x<<" y "<<position.y<<" z "<<position.z<<" conf "<<conf<<" joint num "<<jointNum<<endl;

            msg.position.at(jointNum) = position;//check shavad ke dorost kar mikone ya na
            //   msg.orientation.at(jointNum) = orientation;
            jointNum += 3;
            // cout<<" i @@@@@@@@@@@@@@@@@@@@"<<i<<" size "<<strs.size()<<" 170khune "<<strs.at(170)<<" 171khune " <<strs.at(171)<<endl;
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
            // cout<<" w= "<<orientation.w<<" x "<<orientation.x<<" y "<<orientation.y<<" z "<<orientation.z<<" z "<<endl;
            ++i;//inja ye ++i dge ham bayad bezaram? baraye conf dovvom
            position.x = boost::lexical_cast<double>(strs.at(++i));
            position.y = boost::lexical_cast<double>(strs.at(++i));
            position.z = boost::lexical_cast<double>(strs.at(++i));
            float conf = boost::lexical_cast<float>(strs.at(++i));
            // cout<<" x "<<position.x<<" y "<<position.y<<" z "<<position.z<<" conf "<<conf<<endl;
            // return msg;
            msg.confidence.push_back(conf);
            msg.orientation.push_back(orientation);
            msg.position.push_back(position);
        }
    }
    return msg;
}
