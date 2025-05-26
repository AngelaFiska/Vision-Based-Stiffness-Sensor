#ifndef MY_TRACKING_H
#define MY_TRACKING_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <XmlRpc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/console.h>

namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;



class my_tracking;
class my_tracking
{
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Publisher pubi;
    image_transport::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher MatForce;
    ros::Publisher delta_x;
    cv_bridge::CvImagePtr cv_ptr;
    XmlRpc::XmlRpcValue dist;
    XmlRpc::XmlRpcValue cam;

public:

    /** General Variables*/

    int H_MAX, H_MIN, V_MAX, V_MIN, S_MAX,S_MIN,lower_th,upper_th, MAX_R;

    //Camera Matrix

    cv::Point centroid;
    float rad, force, deltaX;
    std_msgs::Float32 msg;
    std_msgs::Float32 ms_force;
    std_msgs::Float32 ms_delta_x;
    double R_0;
    double X_0,K;

    bool useMorphOps, disp_hough;
    int POS, wp, pre_wpc, white_pc;

    vector<Point> whites, white_circle;
    Mat cameraFeed, HSV,threshold,temp,canny, Mwhite;
    vector< vector<Point> > contours;

    std::vector<double> vect_cam;

    std::vector<double> vect_dist;

    vector<cv::Vec3f> circles;
    Point2f cc;
    Point p;
    Point cp;
    my_tracking();
    ~my_tracking();
    void morphOps(Mat &thresh);
    void image_processing();
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);


};

#endif // MY_TRACKING_H
