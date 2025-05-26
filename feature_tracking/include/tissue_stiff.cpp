#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <std_msgs/Int16.h>
#include<std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <boost/thread.hpp>
#include <boost/bind/bind.hpp>


float delta_xhard;
float delta_xsoft;
float F_soft;
float F_hard;
float delta_tissue;
float k_soft=40;
float k_hard=190;
float kt=0;

class stiff_node
{

public:
 static void callback(const std_msgs::Float32ConstPtr &msg,int a);
 void pubblish_value(ros::NodeHandle nh);

};

void stiff_node::callback(const std_msgs::Float32ConstPtr &msg,int a)
{
    switch (a)
    {
    case  1:
        delta_xsoft=msg->data;
        if(delta_xsoft!=0 && fabs(delta_xsoft)>0.19)
            F_soft= k_soft*delta_xsoft;
        else
        {
            F_soft=0;
            delta_xsoft=0;
        }
        ROS_INFO("deltaxs %f\n",delta_xsoft);
        break;
    case  2:
        delta_xhard=msg->data;
        if (delta_xhard!=0 && fabs(delta_xhard)>0.06)
            F_hard= k_hard*delta_xhard;
        else
        {
            F_hard=0;
            delta_xhard=0;
        }

        ROS_INFO("deltaxR %f\n",delta_xhard);
        delta_tissue=delta_xsoft-delta_xhard;

        ROS_INFO("deltax_TISSUE %f\n",delta_tissue);

        if(delta_tissue!=0){

            kt=fabs((k_hard*delta_xhard-k_soft*delta_xsoft)/delta_tissue);

            }
        else kt=0;

        break;
    }
    ROS_INFO("F_SOFT %f: \n",F_soft);
    ROS_INFO("F_HARD %f: \n",F_hard);

    ROS_INFO("KT %f:\n",kt);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracking");
    ros::NodeHandle nh;
    std_msgs::Float32 ms_tiss_st;
    std_msgs::Float32 ms_Fsoft;
    std_msgs::Float32 ms_Fhard;

    ros::Subscriber sub_dxsoft =nh.subscribe<std_msgs::Float32>("/filtered/delta_xsoft",1,boost::bind(stiff_node::callback,_1,1));
    ros::Subscriber sub_dxhard =nh.subscribe<std_msgs::Float32>("/filtered/delta_xhard",1,boost::bind(stiff_node::callback,_1,2));


    //stiff.pubblish_value(nh);
    ros::Publisher tissue_stiff =nh.advertise<std_msgs::Float32>("/tissue_stiffness",100);
    ros::Publisher tissue_stiff_l =nh.advertise<std_msgs::Float32>("/tissue_Fsoft",100);
    ros::Publisher tissue_stiff_r =nh.advertise<std_msgs::Float32>("/tissue_Fhard",100);

    ros::Rate looprate(30);

    while(ros::ok()){
        ms_tiss_st.data=kt;
        ms_Fsoft.data=F_soft;
        ms_Fhard.data=F_hard;
        tissue_stiff.publish(ms_tiss_st);
        tissue_stiff_l.publish(ms_Fhard);
        tissue_stiff_r.publish(ms_Fsoft);
        ros::spinOnce();
        looprate.sleep();
    }
}
