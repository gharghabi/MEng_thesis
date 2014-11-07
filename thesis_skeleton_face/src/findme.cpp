#include "Config.h"
#include "FindMe.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <cstdio>
#include <iostream>
#include <ctime>
#include <stdexcept>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>


#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <climits>
#include <boost/thread.hpp>

// #include "athomerobot/omnidata.h"

#define START_AFTER_FRAME 2
#define THRESHOLD .55
#define GO_NEAR_THRESHOLD .4
#define LENGTH 1
#define DEPTH_TRESHOLD 0.2

#define TURN_TIME 50
#define PROCESSING_PORT 7788


#define LOGIC_TO_TASK "AUTROBOT_logic_to_find_me"
#define TASK_TO_LOGIC "AUTROBOT_from_find_me_to_logic"



#define PHASE0 "Init"
#define PHASE1 "wait_for_person_come_near"
#define PHASE1_DONE "person_came"
#define PHASE1_STOP "stop_learn"

#define PHASE2 "find_person"
#define PHASE2_DONE "person_found"
#define PHASE2_NotDONE "person_not_found"


#define PHASE3 "find_person_with_gesture"
#define PHASE3_DONE "found_person_with_gesture"
#define PHASE3_NotDONE "person_not_found_with_gesture"


using tld::Config;
using tld::Settings;
using namespace cv;

void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) ;
void rosDepthCallBack(const sensor_msgs::ImageConstPtr &msg) ;
void logicCallBack(const std_msgs::String &msg) ;
void CompassCallBack(const std_msgs::Int32 &msg);
void init();
void logic();
// void Omnidrive(int x, int y, int w);
void scaleRect(cv::Rect &rect, float factor);
bool updateProcessingImage(cv::Mat &processingImage, float depth);
bool getGesture(cv::Rect faceBound);
float average(cv::Mat depth, cv::Rect rect);
bool TurnOk();
bool FaceDetect(int,int);
bool ATMIDDLE(Rect face);
int findMaximumFace();

cv::Mat blackedFaceImage;
cv::Mat camImage;
cv::Mat camDepth;
std::vector<cv::Rect> faces;
FindMe findMe[3];

int PersonNum = -1;//??????????/yadet bashe -1 esh koni
// ros::Publisher omniDrivePublisher;
ros::Publisher logicPublisher;

int state = -1;
int frameCounter = 0;
int learnCounter = 0;
int argc;
char **argv;

bool inited = false;
std::vector<cv::Scalar> colors;

class timer {
	private:
		unsigned long begTime;
	public:
		void start() {
			begTime = clock();
		}

		unsigned long elapsedTime() {
			return ((unsigned long) clock() - begTime) / CLOCKS_PER_SEC;
		}


		bool isTimeout(unsigned long seconds) {
			ROS_INFO("time {begtime:%d, seconds:%d, elapsedtime: %d}.",begTime,seconds,elapsedTime());
			return seconds < elapsedTime();
		}
};

timer tTurn;
timer tWaste;//TODO time waste hesab nashode bayad hesab she

int main(int mainArgc, char **mainArgv) {
    ros::init(mainArgc, mainArgv, "FindMeCocktail");
    ROS_INFO("FindMeCocktail Started.");
    argc = mainArgc;
    argv = mainArgv;

    ros::NodeHandle nodeHandleForLogic;
    ros::NodeHandle nodeHandleForOmniDrive;
    ros::NodeHandle advertiseNodeHandle;
    ros::NodeHandle nodeHandleCompass;


    // omniDrivePublisher = nodeHandleForOmniDrive.advertise<athomerobot::omnidata>("/AUTROBOTIN_omnidrive", 1);

    ros::NodeHandle nodeHandleForFindMeLogicSubscribe;
    ros::NodeHandle nodeHandleForFindMeLogicAdvertise;
    ros::Subscriber taskLogicSubscriber = nodeHandleForFindMeLogicSubscribe.subscribe(LOGIC_TO_TASK, 10, logicCallBack);
    logicPublisher = advertiseNodeHandle.advertise<std_msgs::String>(TASK_TO_LOGIC, 10);


    ros::NodeHandle nodeHandleForKinect;
    image_transport::ImageTransport imageTransport(nodeHandleForKinect);
    image_transport::Subscriber imageSubscriber;
    imageSubscriber = imageTransport.subscribe( "/camera/rgb/image_color", 1, rosImageCallBack);

    ros::NodeHandle nodeHandleForDepth;
    image_transport::ImageTransport imageTransportForDepth(nodeHandleForDepth);
    image_transport::Subscriber depthSubscriber;
    depthSubscriber = imageTransportForDepth.subscribe( "/camera/depth/image", 1, rosDepthCallBack);


    for ( int i = 0; i < 25; i++ ) {
        int r = (rand() % 25 + 1) * 10;
        int g = (rand() % 25 + 1) * 10;
        int b = (rand() % 25 + 1) * 10;
        colors.push_back(cv::Scalar(r, g, b));
    }

    ros::spin();
    return EXIT_SUCCESS;
}


void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) {
    if (!inited) {
        return;
    }
    cv_bridge::CvImagePtr imagePointer;
    try {
        imagePointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imagePointer->image.copyTo(camImage);
	//**********************************************/
    if (frameCounter > START_AFTER_FRAME && camImage.size().width != 0 ) {
        logic();
    } else {
        frameCounter++;
        if (frameCounter == START_AFTER_FRAME) {
            ROS_INFO("Starting Logic");
        }
    }
}


void rosDepthCallBack(const sensor_msgs::ImageConstPtr &msg) {
    if (!inited) {
        return;
    }
    cv_bridge::CvImagePtr imagePointer;
    try {
        imagePointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imagePointer->image.copyTo(camDepth);
}


void logicCallBack(const std_msgs::String &msg) {
    ROS_INFO("logicCallBack: msg: %s", msg.data.c_str());
    if (msg.data=="0") {
        PersonNum=0;
    }
    else if (msg.data=="1") {
        PersonNum=1;
    }
    else if (msg.data=="2") {
        PersonNum=2;
    }
    else if (msg.data == PHASE0 ) {
        state = 0;
        init();        
    } else if (msg.data == PHASE1 ) {
        ROS_INFO("wait for person");
        state = 1;//wait for  person and learn him
    } else if (msg.data == PHASE1_STOP) {
        ROS_INFO("stop learn person");
        state = 2;
    } else if (msg.data == PHASE2) {
    	ROS_INFO("find person");
    	tTurn.start();
        state = 3;
    } else if (msg.data == PHASE3) {
 		ROS_INFO("find person whith gesture");
        // Omnidrive(0,0,50);
 		tTurn.start();
        state = 5;
    }
}
void init() {
    if (inited) {
        return;
    }
    inited = true;
    Config config;
    if (config.init(argc, argv) == PROGRAM_EXIT) {
        exit(EXIT_FAILURE);
    }

    config.configure((&findMe[0]));
	config.configure((&findMe[1]));
	config.configure((&findMe[2]));
    //config.configure(&findMe[3]);
    ROS_INFO("TLD configured.");

}

void logic() {
    cv::Rect result(0, 0, 0, 0);
    std_msgs::String msg;
    if(state == 1)//wait for face
    {
    	ROS_INFO("wait for person: %d",PersonNum);
		if (FaceDetect(120,120))
		{
			msg.data = PHASE1_DONE;
        	logicPublisher.publish(msg);	
        	(&findMe[PersonNum])->updateImage(camImage);
            (&findMe[PersonNum])->detectFace();
        	// int index=findMaximumFace();
        	// (&findMe[PersonNum])->setTrackObjectBoundingBox(&faces[index]);
            (&findMe[PersonNum])->doWork();
	        faces.clear();
	        state= 7;
		}
    }
    else if(state ==2 )//stop learning
	{
		ROS_INFO("stop learning: %d",PersonNum);
		(&findMe[PersonNum])->stopLearning();
		state = -1;    		
	}
    else if(state == 3)//find with tld
	{
		ROS_INFO("find me: %d",PersonNum);
    	(&findMe[PersonNum])->updateImage(camImage);
        (&findMe[PersonNum])->doWork();
        bool confident = ((&findMe[PersonNum])->getRect(result) > THRESHOLD);
        if (confident) {
            int deltaX = (result.x + result.width / 2) - 320 ;
            if (abs(deltaX) < 50) {
            	ROS_INFO("middle: %d",PersonNum);
            	state = 4;

            } else {
                // Omnidrive(0, 0, deltaX / 3);
                ROS_INFO("charkhesh until middle: %d",PersonNum);
            }
        } else {

            if (tTurn.isTimeout(TURN_TIME))//???in ghesmat ke momkene tld ye bar confidentesh bera bala vali vasate nazdik shodan yeho gmesh kone zamane talaf shodasho hesab nakardam albate ajibe age tld peda karde bashe tarafo bad yeho gomesh kone
            {
            	ROS_INFO("time out tld: %d",PersonNum);
                // Omnidrive(0, 0, 0);
                msg.data = PHASE2_NotDONE;
		        logicPublisher.publish(msg);
		        state = -1;
            }
            else
             {
             	ROS_INFO("charkhesh: %d",PersonNum);
				// Omnidrive(0, 0, 50);
             }   
        }
	}
    else if(state == 4)//go near the person
	{
		ROS_INFO("go near the person: %d",PersonNum);
		(&findMe[PersonNum])->updateImage(camImage);
    	(&findMe[PersonNum])->doWork();
    	bool confident = ((&findMe[PersonNum])->getRect(result) > GO_NEAR_THRESHOLD);
    	if (confident) {
        	 ROS_INFO("confident in state 4 go near");        
        	 scaleRect(result, .7);
        	 float depth = average(camDepth, result);
        	if (isnan(depth)) {
            	// Omnidrive(1 * 100, 0, 0);
        	} else {
            float delta = depth - LENGTH;
            ROS_INFO("Lenght to go is: %f", delta);
            if (fabs(delta) < .4) {
                ROS_INFO("Ok, Near the person");
                 // Omnidrive(0, 0, 0);
                msg.data = PHASE2_DONE;
		        logicPublisher.publish(msg);
                state = -1;
            } else {
                ROS_INFO("Go nearest, Depth is %f", depth);
                // Omnidrive(delta * 180, 0, 0);
         	   }
        	}
    	} else {
        	state = 3;/*go back and find again*/
        	ROS_INFO("go back and find again: %d",PersonNum);
    	}
	}
	else if(state ==5 )//find with gesture ??nazdik shodan be fard dar in ghesmat ezafe nashode ast
	{
	 if (!tTurn.isTimeout(TURN_TIME))
     {
     		ROS_INFO("find with gestuer: %d",PersonNum);
        	faces.clear();
            camImage.copyTo(blackedFaceImage);
            if (FaceDetect(20,20));
            {
                for( size_t i = 0; i < faces.size(); i++ )
                    {
                       rectangle(blackedFaceImage,faces[i],(0,0,0),CV_FILLED);
                    }
                for( size_t i = 0; i < faces.size(); i++ )
                	{
                     if (getGesture(faces[i])) {
                     	ROS_INFO("found gesture: %d",PersonNum);	
                            int deltaX = ((faces[i]).x + (faces[i]).width / 2) - 320 ;
                            if (abs(deltaX) < 50) {
                      	     // Omnidrive(0, 0,0);
                             state = 6;

                            ROS_INFO("middle");        

                            } else {//                                Omnidrive(0, 0, deltaX / 3);
                                // Omnidrive(0, 0, deltaX/3);
                                deltaX=abs(deltaX);
                                boost::this_thread::sleep(boost::posix_time::milliseconds(deltaX*7));
                                // Omnidrive(0,0,0); 
                                state = 6;//TO DO 

                                   }
                     }      
                  Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
                  ellipse( camImage, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);   
                };
            }
                imshow("salam",camImage);
                waitKey(30);                             
     }
        // imshow("faceblack",blackedFaceImage);
        // waitKey(10);
     else
     	{
     	  ROS_INFO("not found gesture: %d",PersonNum);
          // Omnidrive(0,0,0);
          msg.data = PHASE3_NotDONE;
          logicPublisher.publish(msg);
          state = -1;
      	}
	}
	else if(state == 6)
		{
			ROS_INFO("go near gesture: %d",PersonNum);
			faces.clear();
            camImage.copyTo(blackedFaceImage);
            if (FaceDetect(20,20));
            {
                for( size_t i = 0; i < faces.size(); i++ )
                    {
                       rectangle(blackedFaceImage,faces[i],(0,0,0),CV_FILLED);
                    }
                for( size_t i = 0; i < faces.size(); i++ )
                	{
                     if (ATMIDDLE(faces[i]))//??????///gestureo Bkhial sho
                     	{
							 ROS_INFO("gesture in state 6 go near");   
							 Rect result;
							 result=faces[i];     
				        	 scaleRect(result, .7);
				        	 float depth = average(camDepth, result);
				        	if (isnan(depth)) 
				        	{
				            	// Omnidrive(1 * 100, 0, 0);
				        	} 
				        	else
				        	{
					            float delta = depth - LENGTH;
					            ROS_INFO("Lenght to go is: %f", delta);
					            if (fabs(delta) < .3)
					            {
					                ROS_INFO("Ok, Near the person");
					                msg.data = PHASE3_DONE;
							        logicPublisher.publish(msg);
							        // Omnidrive(0, 0, 0);
							       	boost::this_thread::sleep(boost::posix_time::milliseconds(100)); 
					                state = -1;
					                ros::shutdown();
					            } 
					            else 
					            {
					                	ROS_INFO("Go nearest, Depth is %f", depth);
					                	// Omnidrive(delta * 180, 0, 0);
					         	}
				        	}
	                 	}      
                  Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
                  ellipse( camImage, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);   
                };
            }
        }
        else if(state ==7 )// learning
        {
        	(&findMe[PersonNum])->updateImage(camImage);
	        (&findMe[PersonNum])->doWork();
	
        }

}

float average(cv::Mat depth, cv::Rect rect) {
    float sum = 0;
    int count = 0;
    float temp;
    for (int x = rect.x; x < rect.x + rect.width; x++) {
        for (int y = rect.y; y < rect.y + rect.height; y++) {
            temp = depth.at<float>(y, x);
            
            if (!isnan(temp)) {
                count++;
                sum += temp;

            }
        }
    }
       
    if (count) {
        return sum / count;
    } else {
        return NAN;
    }
}
void scaleRect(cv::Rect &rect, float factor) {
    int w = rect.width;
    int h = rect.height;
    rect.width *= factor;
    rect.height *= factor;
    rect.x += (w - rect.width) / 2;
    rect.y += (h - rect.height) / 2;
}


bool updateProcessingImage(cv::Mat &processingImage, float depth) {
    cv::Mat depthMat(camDepth.size(), CV_8UC1);
    float temp;
    for (int x = 0; x < camDepth.size().width; x++) {
        for (int y = 0; y < camDepth.size().height; y++) {
            temp = camDepth.at<float>(y, x);
            if (!isnan(temp) && fabs(temp - depth) < DEPTH_TRESHOLD) {
                temp = ((temp - depth - DEPTH_TRESHOLD ) * 255.0 / (2 * DEPTH_TRESHOLD));
            } else {
                temp = 0;
            }
            depthMat.at<char>(y, x) = temp;
        }
    }
    cv::medianBlur(depthMat, depthMat, 31);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( depthMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    if (contours.size() == 0 ) {
        return false;
    }
    for ( int i = 0; i < contours.size(); i++ ) {
        cv::drawContours( processingImage, contours, i, colors[i % 25], CV_FILLED );
    }
    return true;
}

bool getGesture(cv::Rect faceBound) {
    ROS_INFO("getGesture1");
    if (camDepth.size().width == 0) {
        return 0;
    }


// CvScalar  hsv_min = cvScalar(0, 30, 60);
// CvScalar  hsv_max = cvScalar(20, 150, 255);
//range I am using is { 0,30,60,0 & 20,150,255,0 }

    scaleRect(faceBound, .7);
    float depth = average(camDepth, faceBound);
    cv::Mat processingImage = cv::Mat::zeros( camDepth.size(), CV_8UC3 );
  
      
    if (!updateProcessingImage(processingImage, depth)) {
        return 0;
    }
 
  
    cv::Point faceCenter(faceBound.x + faceBound.width / 2, faceBound.y + faceBound.height / 2);
    // find person color
    cv::Vec3b faceColor = processingImage.at<cv::Vec3b>(cv::Point(faceCenter.x, faceCenter.y));
    int yMin = INT_MAX;
    int xMin = INT_MAX;
    int xMax = 0;
    int yMax = 0;

    Rect BBForWave;

    BBForWave.x = faceBound.x - 3 *faceBound.width;
    BBForWave.width = 7 *faceBound.width;
    BBForWave.y = faceBound.y - 3 *faceBound.height;
    BBForWave.height = 4 *faceBound.height;

	if(BBForWave.x<0)
		BBForWave.x=0;
	if(BBForWave.x+BBForWave.width>camImage.cols)
		BBForWave.width=camImage.cols- BBForWave.x;
	if(BBForWave.y<0)
		BBForWave.y=0;
	if (BBForWave.y+BBForWave.height>camImage.rows)
	{
		BBForWave.height=camImage.rows- BBForWave.y;
	}
   // std::cout<<BBForWave.x<< " =x "<<BBForWave.x+BBForWave.width<<" = width"<<camImage.cols<<" =cols "<< BBForWave.y<< " =y "<<BBForWave.y+BBForWave.height<<" height "<<camImage.rows<<" rows "<<std::endl;
    if ((BBForWave.x>-1)&&(BBForWave.x+BBForWave.width<=camImage.cols)&&(BBForWave.y>-1)&&(BBForWave.y+BBForWave.height<=camImage.rows))
    {
     //   ROS_INFO("in");
        Mat hsv_image;
        Mat hsv_mask;
        cv::Mat croppedImage = blackedFaceImage(BBForWave);
        cvtColor(croppedImage, hsv_image, CV_BGR2HSV);
        inRange (hsv_image, Scalar(0, 30, 60), Scalar(20, 150, 255), hsv_mask);
        dilate(hsv_mask, hsv_mask, Mat(),Point(-1,-1),1);
        erode(hsv_mask, hsv_mask, Mat(), Point(-1, -1), 1); // Erode with a 30 x 30 kernel 
 //       ROS_INFO("1");
        
        cv::medianBlur(hsv_mask, hsv_mask, 31);
        imshow("hsv_mask",hsv_mask);
        waitKey(30);

         rectangle(processingImage,Point(BBForWave.x,BBForWave.y),Point(BBForWave.x+BBForWave.width,BBForWave.y+BBForWave.height),(0,0,255),2);

        cv::Vec3b pixel;
        cv::Vec3b pixelmask;
        bool foundPersonColor = false;
        scaleRect(faceBound,1.3);
     
        for (int i = BBForWave.y ; i < BBForWave.height ; i++ ) {
            for ( int j = BBForWave.x ; j < BBForWave.width ; j++) {
                  
                pixel = processingImage.at<cv::Vec3b>(i, j);
                pixelmask =hsv_mask.at<cv::Vec3b>((i-BBForWave.y), (j-BBForWave.x));
    //            printf("H: %d S: %d V: %d\n", pixelmask[0],pixelmask[1],pixelmask[2]);
             //  std::cout<<(int)(pixelmask[0])<<" "<<(int)(pixelmask[1])<<" "<<(int)(pixelmask[2])<<std::endl;
                if ((pixel[0] == faceColor[0]) && (pixel[1] == faceColor[1]) && (pixel[2] == faceColor[2])) {
                    // in range pyda shode atrafe sare fard bude va tu hamun fasele gharar dashte bashad ba rbg moshkel daram
                   
                        if (((int)(pixelmask[0])!=0)||((int)(pixelmask[1])!=0)||((int)(pixelmask[2])!=0))
                        {
                            foundPersonColor = true;
                            if (i < xMin) {
                                xMin = i;
                            }
                            if (j < yMin) {
                                yMin = j;
                            }
                            if (i > xMax) {
                                xMax = i;
                            }
                            if (j > yMax) {
                                yMax = j;
                            }   
                         }
                }
            }
        }
     
     if ((xMin!=INT_MAX)&& (yMin!=INT_MAX))
     {  
        rectangle(processingImage,cvPoint(xMin,yMin), cvPoint(xMax, yMax ),(255,0,0),2);
     }
        cv::imshow( "Contours", processingImage );
        cv::waitKey(10);
        
        std::cout<<"logicCallBack: msg: "<<foundPersonColor<<std::endl;

    
    return foundPersonColor;
    }
return false;
}
bool TurnOk()
{
    return true;
}

bool ATMIDDLE(Rect face)
{
	if ((((face.x+face.width)/2)>100)&&((face.x+face.width)/2)<500)
		return true;
	else
		return false;
}

bool FaceDetect(int a, int b)
{
			  String face_cascade_name = "/home/athome/catkinws/src/athomerobot/find_me/haarcascade_frontalface_default.xml";
	          
              cv::CascadeClassifier face_cascade;
              face_cascade = cv::CascadeClassifier(face_cascade_name);
              Mat gray;
              cvtColor(camImage,gray,CV_BGR2GRAY);
              if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); };
                 face_cascade.detectMultiScale( gray, faces, 1.2, 12, 0|CV_HAAR_SCALE_IMAGE, Size(a, b) );//khub ba yek ghalat gray, faces, 1.2, 10, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) )
				for( size_t i = 0; i < faces.size(); i++ )
                	{
				Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
                  ellipse( camImage, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);   
              }
              imshow("salam",camImage);
              waitKey(20);

			if (faces.size()!=0)
			{
				return true;
			}
			else 
				return false;

}
int findMaximumFace() {
    if (faces.empty()) {
        return -1;
    }
    int maximumIndex = 0;
    float maximum;
    maximum = faces[0].width * faces[0].height ;
    float temp;
    for (int i = 1; i < faces.size(); i++) {
        if ((temp = faces[i].width * faces[i].height) > maximum) {
            maximumIndex = i;
            maximum = temp;
        }
    }
    ROS_INFO("max index %d",maximumIndex);
    return maximumIndex;
}

