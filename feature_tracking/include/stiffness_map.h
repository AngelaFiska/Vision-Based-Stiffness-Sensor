#ifndef STIFFNESS_MAP_H
#define STIFFNESS_MAP_H
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <std_msgs/Int16.h>
#include <XmlRpc.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace cv;
using namespace message_filters;

class stiffness_map;
class stiffness_map
{
    ros::NodeHandle nh;
    ros::Subscriber sub_r;
    ros::Subscriber sub_l;
    ros::Publisher pub_l;
    ros::Publisher pub_r;

    
public:
	
    int previous_radr,previous_radl;
    float alfa;
    float tau;
    float sample_time;

    void stiffCallback(const ros::MessageEvent<std_msgs::String const> & event);
    void compute_map();
	


};

#endif //STIFFNESS_MAP_H
