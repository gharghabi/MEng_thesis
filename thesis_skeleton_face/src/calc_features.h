#include <ros/ros.h>
#include "upperbodycore_msgs/Skeleton.h"
struct position
{
	double x;
	double y;
	double z;
};
struct floor_coefficient
{
 	double a;
	double b;
	double c;
	double d;
	double normal;
};
class calc_features
{

	private:
	    	    
	    static int const jointNum =15;
        static int const features_count = 16;
	    ros::NodeHandle nh_;
	    ros::Subscriber sub_;
  	    float confidence;
	    floor_coefficient floorParameters;
            double body_features[features_count];
        double normalize_body_features[features_count];
	    void Calc_All_Features();

	public:
  	    upperbodycore_msgs::Skeleton SkData;

	    calc_features();
	   ~calc_features();
	   void set_floor(double a, double b, double c, double d);
	   double *get_features_skeleton_article();
 	   double get_R_torso_leg();
 	   double get_ED_floor_neck();
	   double get_ED_floor_head();
 	   double get_hight_estimate();
	   double *get_normalize_features();
           double get_ED_neck_leftshoulder();
  	   double get_ED_neck_rightshoulder();
   	   double *calc_variance_average(double *sub_bodyfeatures, int arrayLength);
       	   void SkeletonData_CB(const upperbodycore_msgs::Skeleton msg);
	   double *calc_normalize_features(double *personBodyFeatures);
	   double *get_features_skeleton_my();
	};
