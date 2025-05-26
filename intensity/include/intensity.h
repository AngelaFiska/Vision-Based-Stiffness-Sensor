/*
 * intensity.h
 *
 *  Created on: Jul 25, 2012
 *      Author: mende
 */

#ifndef INTENSITY_H_
#define INTENSITY_H_

#include "DeckLinkAPI.h"
#include "capture.h"
#include <sensor_msgs/SetCameraInfo.h>
#include <intensity_paramsConfig.h>

using namespace std;
using namespace cv;
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

class intensity_class
{

	public:
		intensity_class(ros::NodeHandle intensity_nh);
		~intensity_class();

		// routines that help to identify DeckLink card's capabilities
		void	print_attributes (IDeckLink* deckLink);
		void	print_output_modes (IDeckLink* deckLink);
		void	print_capabilities (IDeckLink* deckLink);

		int		CheckDevices ();
		bool 	StartStreaming ();
		bool 	StopStreaming ();
		int 	usage(int status);

		image_transport::CameraPublisher 		pub;
		image_transport::CameraPublisher 		pub_distorted;

		std::string camera_name;

		DeckLinkCaptureDelegate 		*delegate;

		void configCallback(intensity::intensity_paramsConfig &config, uint32_t level);

private:
		HRESULT							result;
		IDeckLinkInput					*deckLinkInput;
		IDeckLinkDisplayModeIterator	*displayModeIterator;
		IDeckLinkDisplayMode			*displayMode;
		BMDDisplayMode					selectedDisplayMode;
		BMDPixelFormat					pixelFormat;
		BMDVideoInputFlags				inputFlags;
		BMDTimecodeFormat				g_timecodeFormat;
		int								g_videoModeIndex;
		int								g_audioChannels;
		int								g_audioSampleDepth;
		char *							g_audioOutputFile;
		int								g_maxFrames;
		int								displayModeCount;
		bool 							foundDisplayMode;
		IDeckLinkIterator				*deckLinkIterator;
		IDeckLink						*deckLink;
		int								numDevices;
		int								videoOutputFile;
		unsigned long 					frameCount;

		bool 							exitStatus;


};

#endif /* INTENSITY_H_ */
