#ifndef INTENSITY_TRACK_H
#define INTENSITY_TRACK_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include <XmlRpc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/console.h>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"


namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;



class intensity_track;
class intensity_track
{
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Publisher pubi;
    image_transport::Subscriber sub;
    ros::Publisher pubrm1,pubrm2,pubrm1s,pubrm2s, radius_roi1, radius_roi2,radius_roi3,  radius_roi4, cen1_pub,cen2_pub,cen3_pub, cen4_pub;


    ros::Publisher Force1,Force2,Force3,Force4, delta_x1,delta_x2, delta_x3, delta_x4,pub_ks1,pub_ks2,pub_ks4,pub_ks3 ;

        std::vector<ros::Publisher> c_pub;
    XmlRpc::XmlRpcValue dist;
    XmlRpc::XmlRpcValue cam;

public:
    cv_bridge::CvImagePtr cv_ptr;

    /** General Variables*/

    int H_MAX, H_MIN, V_MAX, V_MIN, S_MAX,S_MIN;
    int lower_th,upper_th, MAX_R;

    //Camera Matrix

    cv::Point centroid1,centroid2,centroid3,centroid4;
    float rad1,rad2, rad3,rad4,forces,forceh,force1,force2,force3,force4, deltaX1,deltaX2,deltaX3,deltaX4,mean_r1,mean_r2,deltaX1s,deltaX2s,ks1,ks2,ks3,ks4;
    float frad1,frad2,frad3,frad4, pre_rad1, pre_rad2, pre_rad3, pre_rad4,pre_f1;
    std_msgs::Float32 mradius1, mradius2, mradius3,mradius4, msradius1,msradius2,msradius4,msradius3;

    std_msgs::Float32 ms_force1,ms_force2,ms_force3,ms_force4,ms_ks1,ms_ks2,ms_ks3,ms_ks4;
    std_msgs::Float32 ms_delta_x1,ms_delta_x2,ms_delta_x3,ms_delta_x4;

geometry_msgs::Point cen1,cen2,cen3,cen4;
    double R1_0,R2_0;
    double X_0,K1,K2,MAX_R1,MAX_R2,R_10,R_20,h;
    double R3_0,R4_0;
    double X4_0,X3_0,R_30,R_40;
    float p0,p12,p13,p14,p11;
    bool useMorphOps, disp_hough;
    int POS, wp1,wp2,wp3,wp4, pre_wpc, white_pc;

    vector<Point> whites1, white_circle1,whites2, white_circle2,whites3, white_circle3,whites4, white_circle4;

    Mat cameraFeed, HSV,threshold,temp,canny, M1white, M2white,M3white,M4white;
    vector< vector<Point> > contours;

    std::vector<double> vect_cam;

    std::vector<double> vect_dist;

    vector<cv::Vec3f> circles;
    Point2f cc1,cc2,cc3,cc4;
    Point p1,p2,p3,p4;
    Point cp1,cp2,cp3,cp4;


    intensity_track();
    ~intensity_track();
    void morphOps(Mat &thresh);
    void image_processing();
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);


};

#endif // intensity_track_H
