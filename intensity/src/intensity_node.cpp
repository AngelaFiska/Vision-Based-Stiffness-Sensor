/*
 * intensity.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: mende
 */

#include "std_msgs/String.h"
#include "intensity.h"
#include "image_transport/image_transport.h"

// Dynamic reconfigure includes.
#include "dynamic_reconfigure/server.h"
// Auto-generated from cfg/ directory.
#include <intensity_paramsConfig.h>

extern int ratereduction;
extern int resolution;
extern std::string intensity_path;

using namespace intensity;

/**
 * This node is responsible for taking images from the Intensity Pro frame grabber, do some pre-processing and sending them to ROS
 */

int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "Intensity");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // images are sent via image transport package, see http://www.ros.org/wiki/image_transport for details
  image_transport::ImageTransport it(n);
  image_transport::CameraPublisher pub = it.advertiseCamera("camera/image", 1);
  image_transport::CameraPublisher pub_distorted = it.advertiseCamera("camera/image_distorted", 1);

  // The (very low) loop rate is only necessary to detect configuration changes
  ros::Rate loop_rate(3);

  // frame grabber functionality is outsourced to an "intensity" object
  intensity_class Framegrabber(n);
  Framegrabber.pub=pub;
  Framegrabber.pub_distorted=pub_distorted;

  // This service is needed to receive calibration information
  ros::ServiceServer set_camera_info = n.advertiseService("camera/set_camera_info", setCameraInfo);

  // Check for present frame grabber
  bool bIsDevicePresent;
  bool bIsDeviceStreaming;
  bIsDevicePresent=Framegrabber.CheckDevices();

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<intensity::intensity_paramsConfig> dr_srv;
  dynamic_reconfigure::Server<intensity::intensity_paramsConfig>::CallbackType cb;
  cb = boost::bind(&intensity_class::configCallback, Framegrabber, _1, _2);
  dr_srv.setCallback(cb);

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("ratereduction", ::ratereduction, int(1));
  private_node_handle_.param("resolution", ::resolution, int(2));


  if (bIsDevicePresent)
  {
	  	// is frame grabber ready to receive images?
		bIsDeviceStreaming=Framegrabber.StartStreaming();

		if (bIsDeviceStreaming)
		{
			ROS_INFO("Intensity starts streaming");

			while (ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		//clean up
		Framegrabber.StopStreaming();
  }
  return 0;
}
