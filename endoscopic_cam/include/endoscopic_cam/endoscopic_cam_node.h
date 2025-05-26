#ifndef ENDOSCOPIC_CAM_NODE_H
#define ENDOSCOPIC_CAM_NODE_H
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
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <limits>
#include <string>
#include <endoscopic_cam/Endo_Feat_Locations.h>
#include <dynamic_reconfigure/server.h>
#include <endoscopic_cam/endoscopic_dvarConfig.h>
#include <std_msgs/Float32MultiArray.h>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

struct Params{
    int H_MIN, H_MAX, V_MIN,V_MAX, S_MIN, S_MAX;// HSV parameters
    double resolution ; //camera resolution
    bool useMorphOps;
    std::vector<int> rois_p;
    std::vector<int> rois_size;
};


/* Structure to read the variable from dynamic reconfigure */
struct filt_config{
    double alpha; // Low pass filter parameter tao/T
    double P; // Kalman estimation error covariance
    double x;// Kalman value x
    double y;
    double Q;// Kalman Process noise covariance
    double R; // Kalman measurement noise covariance
    int N;
};

class Img_Proc{

protected:
    int mov_av_i;
    int n_roi;
    Point2d ROI;
    Size ROI_sz;
    Params par;
    Point3d pos;
    Point2d previous_centroid, kalman_x,kalman_y;
    filt_config filters;
    std::vector<Point2d> window;
    double q; //process noise covariance
    double r; //measurement noise covariance
    double k; //kalman gain

public:

    std::vector<double> x,y; // kalman state
    std::vector<double> p; //estimation error covariance
    Img_Proc(int in_roi,Params ipar,Point3d ipos,Point2d icen);
    Point2d computeCentroid(cv::Mat input_im, int n, filt_config f);
    Point3d calculate_3D_point(Point2d point);
    void morphOps(Mat &thresh);
    Point2d raw_centroid, after_low_pass, after_kalman;
    Point2d  centroid_filtered;
    std::vector<double>kalman_centroid ;
    void init_Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value_x,double intial_value_y,int n );
    double getFilteredValue_x(double measurement_x, int n);
    double getFilteredValue_y(double measurement_y, int n);
};



class Endoscopic_Cam_Node
{
protected:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    ros::Publisher pub, ind_pub;
    cv_bridge::CvImagePtr cv_ptr;
    Params load_parameter(std::string );
    ros::Publisher pub_centroids;

public:
    filt_config filt_cfg;
    std::vector<Img_Proc> features;
    std_msgs::Float32MultiArray image_centroids;
    double alpha;
    Endoscopic_Cam_Node();
    ~Endoscopic_Cam_Node();
    void visualization_func(vector<Point3d>,Point3f normal);
    void image_processing(Mat cv_ptr);
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
    Params param;
    dynamic_reconfigure::Server<endoscopic_cam::endoscopic_dvarConfig> server;
    dynamic_reconfigure::Server<endoscopic_cam::endoscopic_dvarConfig>:: CallbackType f;
    void DynamConfcallback(endoscopic_cam::endoscopic_dvarConfig &config, uint32_t level);
    void initialize();

};



#endif //
