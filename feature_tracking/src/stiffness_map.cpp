
#include "stiffness_map.h"

int  start=0;

stiffness_map::stiffness_map()
    : it(nh)
{

    sub_r=nh.subscribe<std_msgs::String>("/my_radiusr",1,&stiffness_map::stiffCallback(),this);
    sub_l=nh.subscribe<std_msgs::String>("/my_radiusr",1,&stiffness_map::stiffCallback(),this);



}
stiffness_map::~stiffness_map()
{


}



void stiffness_map::stiffCallback(const ros::MessageEvent<std_msgs::String const> & event)
{


ros::Time begin = ros::Time::now();
    compute_map();
}


void stiffness_map::compute_map(){



}








int main(int argc, char **argv)
{
    ros::init(argc, argv, "stiffness_map");

    stiffness_map object;
    while (ros::ok())
    {
      ros::r.spin();
    }



}

