#include "calc_features.h"

using namespace std;
calc_features::calc_features()
{
    sub_ = nh_.subscribe("skeleton", 1000, &calc_features::SkeletonData_CB,this);
    SkData.user_id = -1;
    confidence = 0.3;
    for(int i=0; i<features_count;++i)
        body_features[i] = 0;
    cout<<"init done for git"<<endl;
}
calc_features::~calc_features()
{

}
void calc_features::SkeletonData_CB(const upperbodycore_msgs::Skeleton msg)
{
    SkData = msg;
//    cout<<" what "<<endl;
//    cout<<msg.confidence[1]<<"conf"<<endl;
//    cout<<SkData.confidence[1]<<"conf2"<<endl;

    // double *featuresFornormalize = get_features();
    // calc_normalize_features(featuresFornormalize);
}
double *calc_features::get_features_skeleton_article()
{
    if(SkData.user_id!=-1)
    {
        int h=SkData.user_id;

        upperbodycore_msgs::Skeleton normalData;
        normalData = SkData;
//        cout<<"0"<<SkData.position[0].x<<"15 "<<SkData.position[14].z<<endl;
       //  //+_+_+_+_+_+_+_+_+_+_+_+_+_+ 
       //  //dar kole in mohasebat be dalile mohasebe nakardane zamin bejaye maghadire zamin miangine maghadire pa gharar dade shode ast
       // //+_+_+_+_+_+_+_+_+_+_+_+_+_+ 
       //  position EFloor;//estimated floor  
       //  EFloor.x = (SkData.position[11].x + SkData.position[14].x)/2;
       //  EFloor.y = (SkData.position[11].y + SkData.position[14].y)/2;
       //  EFloor.z = (SkData.position[11].z + SkData.position[14].z)/2;


        // double DFLH = abs(SkData.position[9].y - EFloor.y); // destination floorLhip 
        // double DFRH = abs(SkData.position[12].y - EFloor.y); // destination floor Rhip

        double DFLH = abs(floorParameters.a * SkData.position[9].x + floorParameters.b * SkData.position[9].y + floorParameters.c * SkData.position[9].z +floorParameters.d)/floorParameters.normal;
        double DFRH = abs(floorParameters.a * SkData.position[12].x + floorParameters.b * SkData.position[12].y + floorParameters.c * SkData.position[12].z +floorParameters.d)/floorParameters.normal;
        position hipsAverage,kneesAverage,footsAverage;
        hipsAverage.x = (SkData.position[9].x+SkData.position[12].x)/2;
        hipsAverage.y = (SkData.position[9].y+SkData.position[12].y)/2;
        hipsAverage.z = (SkData.position[9].z+SkData.position[12].z)/2;

        kneesAverage.x = (SkData.position[10].x+SkData.position[13].x)/2;
        kneesAverage.y = (SkData.position[10].y+SkData.position[13].y)/2;
        kneesAverage.z = (SkData.position[10].z+SkData.position[13].z)/2;

        footsAverage.x = (SkData.position[11].x+SkData.position[14].x)/2;
        footsAverage.y = (SkData.position[11].y+SkData.position[14].y)/2;
        footsAverage.z = (SkData.position[11].z+SkData.position[14].z)/2;

        body_features[1] = abs(floorParameters.a * SkData.position[0].x + floorParameters.b * SkData.position[0].y + floorParameters.c * SkData.position[0].z +floorParameters.d)/floorParameters.normal;//manzur az faseleye oghlidosi chist dr inja?
        body_features[5] = sqrt(pow(double(SkData.position[3].x - SkData.position[1].x),2)+pow(double(SkData.position[3].y - SkData.position[1].y),2)+pow(double(SkData.position[3].z - SkData.position[1].z),2));
        body_features[6] = sqrt(pow(double(SkData.position[6].x - SkData.position[1].x),2)+pow(double(SkData.position[6].y - SkData.position[1].y),2)+pow(double(SkData.position[6].z - SkData.position[1].z),2));
        body_features[2] = ((body_features[5] + body_features[6])/2)/(((DFLH+DFRH)/2)* body_features[1]);
        body_features[4] = abs(floorParameters.a * SkData.position[1].x + floorParameters.b * SkData.position[1].y + floorParameters.c * SkData.position[1].z +floorParameters.d)/floorParameters.normal;
        body_features[7] = sqrt(pow(double(SkData.position[2].x - SkData.position[6].x),2)+pow(double(SkData.position[2].y - SkData.position[6].y),2)+pow(double(SkData.position[2].z - SkData.position[6].z),2));
        body_features[3] = sqrt(pow(double(SkData.position[0].x - SkData.position[1].x),2)+pow(double(SkData.position[0].y - SkData.position[1].y),2)+pow(double(SkData.position[0].z - SkData.position[1].z),2))
                         + sqrt(pow(double(SkData.position[1].x - SkData.position[2].x),2)+pow(double(SkData.position[1].y - SkData.position[2].y),2)+pow(double(SkData.position[1].z - SkData.position[2].z),2))
                         + sqrt(pow(double(SkData.position[2].x - SkData.position[9].x),2)+pow(double(SkData.position[2].y - SkData.position[9].y),2)+pow(double(SkData.position[2].z - SkData.position[9].z),2))
                         + sqrt(pow(double(SkData.position[9].x - SkData.position[10].x),2)+pow(double(SkData.position[9].y - SkData.position[10].y),2)+pow(double(SkData.position[9].z - SkData.position[10].z),2))
                         + sqrt(pow(double(SkData.position[10].x - SkData.position[11].x),2)+pow(double(SkData.position[10].y - SkData.position[11].y),2)+pow(double(SkData.position[10].z - SkData.position[11].z),2));
    // cout<<" gardan ba shuneye chap "<<sqrt(pow(double(SkData.position[3].x - SkData.position[1].x),2)+pow(double(SkData.position[3].y - SkData.position[1].y),2)+pow(double(SkData.position[3].z - SkData.position[1].z),2))<<" z dist "<<double(SkData.position[3].z - SkData.position[1].z)<<" x dist "<<double(SkData.position[3].x - SkData.position[1].x)<<" y dist "<<endl;
      }
    return body_features;
}
double *calc_features::get_features_skeleton_my()
{
    if(SkData.user_id!=-1)
    {
        int h=SkData.user_id;
        body_features[1] = sqrt(pow(double(SkData.position[0].x - SkData.position[1].x),2)+pow(double(SkData.position[0].y - SkData.position[1].y),2)+pow(double(SkData.position[0].z - SkData.position[1].z),2));
        body_features[2] = sqrt(pow(double(SkData.position[6].x - SkData.position[2].x),2)+pow(double(SkData.position[6].y - SkData.position[2].y),2)+pow(double(SkData.position[6].z - SkData.position[2].z),2));
        body_features[3] = sqrt(pow(double(SkData.position[3].x - SkData.position[2].x),2)+pow(double(SkData.position[3].y - SkData.position[2].y),2)+pow(double(SkData.position[6].z - SkData.position[1].z),2));
        body_features[4] = sqrt(pow(double(SkData.position[0].x - SkData.position[1].x),2)+pow(double(SkData.position[0].y - SkData.position[1].y),2)+pow(double(SkData.position[0].z - SkData.position[1].z),2))
                         + sqrt(pow(double(SkData.position[1].x - SkData.position[2].x),2)+pow(double(SkData.position[1].y - SkData.position[2].y),2)+pow(double(SkData.position[1].z - SkData.position[2].z),2))
                         + sqrt(pow(double(SkData.position[2].x - SkData.position[9].x),2)+pow(double(SkData.position[2].y - SkData.position[9].y),2)+pow(double(SkData.position[2].z - SkData.position[9].z),2))
                         + sqrt(pow(double(SkData.position[9].x - SkData.position[10].x),2)+pow(double(SkData.position[9].y - SkData.position[10].y),2)+pow(double(SkData.position[9].z - SkData.position[10].z),2))
                         + sqrt(pow(double(SkData.position[10].x - SkData.position[11].x),2)+pow(double(SkData.position[10].y - SkData.position[11].y),2)+pow(double(SkData.position[10].z - SkData.position[11].z),2));
        body_features[5] = sqrt(pow(double(SkData.position[1].x - SkData.position[2].x),2)+pow(double(SkData.position[1].y - SkData.position[2].y),2)+pow(double(SkData.position[1].z - SkData.position[2].z),2));
        body_features[6] = sqrt(pow(double(SkData.position[12].x - SkData.position[13].x),2)+pow(double(SkData.position[12].y - SkData.position[13].y),2)+pow(double(SkData.position[12].z - SkData.position[13].z),2))
						 + sqrt(pow(double(SkData.position[13].x - SkData.position[14].x),2)+pow(double(SkData.position[13].y - SkData.position[14].y),2)+pow(double(SkData.position[13].z - SkData.position[14].z),2));
        body_features[7] = sqrt(pow(double(SkData.position[9].x - SkData.position[10].x),2)+pow(double(SkData.position[9].y - SkData.position[10].y),2)+pow(double(SkData.position[9].z - SkData.position[10].z),2))
						 + sqrt(pow(double(SkData.position[10].x - SkData.position[11].x),2)+pow(double(SkData.position[10].y - SkData.position[11].y),2)+pow(double(SkData.position[10].z - SkData.position[11].z),2));
        body_features[8] = sqrt(pow(double(SkData.position[1].x - SkData.position[2].x),2)+pow(double(SkData.position[1].y - SkData.position[2].y),2)+pow(double(SkData.position[1].z - SkData.position[2].z),2))
                + sqrt(pow(double(SkData.position[2].x - SkData.position[9].x),2)+pow(double(SkData.position[2].y - SkData.position[9].y),2)+pow(double(SkData.position[2].z - SkData.position[9].z),2))
                + sqrt(pow(double(SkData.position[9].x - SkData.position[10].x),2)+pow(double(SkData.position[9].y - SkData.position[10].y),2)+pow(double(SkData.position[9].z - SkData.position[10].z),2))
                + sqrt(pow(double(SkData.position[10].x - SkData.position[11].x),2)+pow(double(SkData.position[10].y - SkData.position[11].y),2)+pow(double(SkData.position[10].z - SkData.position[11].z),2));
        body_features[9] = sqrt(pow(double(SkData.position[3].x - SkData.position[1].x),2)+pow(double(SkData.position[3].y - SkData.position[1].y),2)+pow(double(SkData.position[3].z - SkData.position[1].z),2));
        body_features[10]= sqrt(pow(double(SkData.position[6].x - SkData.position[1].x),2)+pow(double(SkData.position[6].y - SkData.position[1].y),2)+pow(double(SkData.position[6].z - SkData.position[1].z),2));
        body_features[11]= sqrt(pow(double(SkData.position[3].x - SkData.position[4].x),2)+pow(double(SkData.position[3].y - SkData.position[4].y),2)+pow(double(SkData.position[3].z - SkData.position[4].z),2))
                         + sqrt(pow(double(SkData.position[4].x - SkData.position[5].x),2)+pow(double(SkData.position[4].y - SkData.position[5].y),2)+pow(double(SkData.position[4].z - SkData.position[5].z),2));
        body_features[12]= sqrt(pow(double(SkData.position[6].x - SkData.position[7].x),2)+pow(double(SkData.position[6].y - SkData.position[7].y),2)+pow(double(SkData.position[6].z - SkData.position[7].z),2))
                         + sqrt(pow(double(SkData.position[7].x - SkData.position[8].x),2)+pow(double(SkData.position[7].y - SkData.position[8].y),2)+pow(double(SkData.position[7].z - SkData.position[8].z),2));
        body_features[13]= sqrt(pow(double(SkData.position[9].x - SkData.position[10].x),2)+pow(double(SkData.position[9].y - SkData.position[10].y),2)+pow(double(SkData.position[9].z - SkData.position[10].z),2));
        body_features[14]= sqrt(pow(double(SkData.position[13].x - SkData.position[12].x),2)+pow(double(SkData.position[13].y - SkData.position[12].y),2)+pow(double(SkData.position[13].z - SkData.position[12].z),2));
        body_features[15]= body_features[5]/body_features[14];
        body_features[16]= body_features[5]/body_features[13];
    }
    return body_features;

}
double calc_features::get_ED_floor_head()
{
    return body_features[1];
}
double calc_features::get_R_torso_leg()//in nist
{
    return body_features[2];
}
double calc_features::get_hight_estimate()
{
    return body_features[3];
}
double calc_features::get_ED_floor_neck()
{
    return body_features[4];
}
double calc_features::get_ED_neck_leftshoulder()
{
    return body_features[5];
}
double calc_features::get_ED_neck_rightshoulder()
{
    return body_features[6];
}
double *calc_features::calc_variance_average(double *sub_bodyfeatures,int arrayLength)
{
//    cout<<"array size "<<arrayLength<<endl;
	double variance_average[2];
	double Ex,Ex2;
	Ex = Ex2 = 0;
    for(int i = 0; i<arrayLength ;++i)
	{
//        cout<<" feature[i] "<<i<<" "<<sub_bodyfeatures[i]<<endl;
		Ex += sub_bodyfeatures[i];
		Ex2 += pow(sub_bodyfeatures[i],2);
	}	
    Ex/=arrayLength;
    Ex2/=arrayLength;
    variance_average[0] = Ex;
    variance_average[1] = sqrt(Ex2 - pow(Ex,2));
    cout<<" miangin "<<variance_average[0]<<" variance "<<variance_average[1]<<endl;

	return variance_average;
}
double *calc_features::calc_normalize_features(double *personBodyFeatures)//in ghalate bayad kole dadehaye amuzeshi ghad miangin va variance anha hesab she
{

	double normalize_data[features_count];
	double *variance_average;
    variance_average = calc_variance_average(normalize_data,features_count);
	for(int i = 0; i<features_count; ++i)
	{
		normalize_data[i] = (normalize_data[i] - variance_average[1])/variance_average[0];
        normalize_body_features[i] = normalize_data[i];
	}
	return normalize_data;
}
double *calc_features::get_normalize_features()
{
    return normalize_body_features;
}
void calc_features::set_floor(double a, double b, double c, double d)
{
    floorParameters.a = a;
    floorParameters.b = b;
    floorParameters.c = c;
    floorParameters.d = d;
    floorParameters.normal = sqrt(pow(floorParameters.a,2)+pow(floorParameters.b,2)+pow(floorParameters.c,2));
 }  
