/*
 * capture.h
 *
 *  Created on: Jul 26, 2012
 *      Author: mende
 */

#define CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))


#ifndef __CAPTURE_H__
#define __CAPTURE_H__

#include "DeckLinkAPI.h"
#include <pthread.h>
#include "image_transport/image_transport.h"
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class DeckLinkCaptureDelegate : public IDeckLinkInputCallback
{
public:
	DeckLinkCaptureDelegate();
	~DeckLinkCaptureDelegate();

	cv::Mat uyvy_to_mat (int width, int height, unsigned char *src);
	void yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray);
	void yuv422ToColor(const cv::Mat& yuv, cv::Mat& color);

	image_transport::CameraPublisher pub;
	image_transport::CameraPublisher pub_distorted;
	sensor_msgs::ImagePtr msg;

	//! corrects lens distortion for the given camera matrix and distortion coefficients
	void undistort( cv::Mat src, cv::Mat dst, cv::Mat cameraMatrix, cv::Mat distCoeffs);
	int counter;

private:
	virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
	virtual ULONG STDMETHODCALLTYPE AddRef(void);
	virtual ULONG STDMETHODCALLTYPE  Release(void);
	virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
	HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

	unsigned long 			frameCount;
	BMDTimecodeFormat		g_timecodeFormat;
	int						g_maxFrames;

	void*					frameBytes;

	ULONG					m_refCount;
	pthread_mutex_t			m_mutex;
	pthread_mutex_t			sleepMutex;
	pthread_cond_t			sleepCond;

};

#endif
