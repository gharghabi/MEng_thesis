
#include "calc_features.h"
#include <boost/thread.hpp>
using namespace std;
//========================================================
/*
   Project description : PGITIC_wave_detect
*/
//==========================================================
// void wavedetect();
// wave *wave_client;
// bool wave_start = false;

calc_features *gClinet;
// bool check_wave_request(upperbodycore_msgs::waveSrv::Request  &req, upperbodycore_msgs::waveSrv::Response &res);

// ros::Publisher wavePublisher;
// upperbodycore_msgs::wave_hand wave_msg;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_features");
    std::cout << "body features started" << std::endl;
    gClinet = new calc_features();
    // gClinet->get_features();
    // wave_client = new wave();
    // // upperbodyclient Client;
    // //gClinet = &Client;
    // ros::NodeHandle nh_,n_;
    // wavePublisher = nh_.advertise<upperbodycore_msgs::wave_hand>("pgitic_wave_detect", 1);
    // ros::ServiceServer service_wave_request = n_.advertiseService("pgitic_wave_request", check_wave_request);
    // boost::thread wave_process(&wavedetect);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    // wave_process.interrupt();
    // wave_process.join();
    return 0;
}


// void wavedetect()
// {
//     while (ros::ok())
//     {
//         if (wave_start)
//         {
//             cout<<"true shod"<<endl;
//             if (wave_client->right_hand_wave_detect())
//             {
//                 wave_msg.right = "true";
//                 std::cout << " right hand wave detected RRRRRRRRRRRRRR " << std::endl;

//                 //             gClinet->setMotor_rightShoulderYaw(1100,5);
//                 //              boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

//                 //                  gClinet->resetMotor_rightArm();
//                 //                   boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
//             }
//             else
//             {
//                 wave_msg.right = "false";
//                 std::cout << " not right" << std::endl;
//             }
//             if (wave_client->left_hand_wave_detect())
//             {
//                 std::cout << " left hand wave detected LLLLLLLLLLLLLLLLLLL" << std::endl;
//                 wave_msg.left = "true";
//             }
//             else
//             {
//                 wave_msg.left = "false";
//                 std::cout << " not left" << std::endl;
//             }
//             wavePublisher.publish(wave_msg);
//         }
//         else
//         {
//             boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//         }
//     }
// }
// bool check_wave_request(upperbodycore_msgs::waveSrv::Request  &req, upperbodycore_msgs::waveSrv::Response &res)
// {
//     cout<<"service call shod"<<endl;
//     if (req.command == "wave_start")
//     {
//         wave_start = "true";
//     }
//     else if (req.command == "wave_stop")
//     {
//         wave_start = "false";
//     }
//     return true;
// }

