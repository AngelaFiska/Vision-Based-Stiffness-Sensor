/*
 * capture.cpp
 *
 *  Created on: Jul 26, 2012
 *      Author: mende
 */


#include "capture.h"
#include "intensity.h"
#include "DeckLinkAPI.h"
#include "omp.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
//#include <vector>

sensor_msgs::CameraInfo camera_info;

using namespace cv;
using namespace std;


int imagedrop=1;
int ratereduction=1;
int resolution=1;
std::string intensity_path;

/*****************************************************************************
 *	Constructor with initialization of decklink properties and creation
 *	of a semaphore
 *****************************************************************************/
DeckLinkCaptureDelegate::DeckLinkCaptureDelegate() : m_refCount(0)
{
	frameCount=0;
	g_timecodeFormat = 0;
	g_maxFrames = -1;
	pthread_mutex_init(&m_mutex, NULL);
	counter=1;
}

/*****************************************************************************
 *	Destructor
 *****************************************************************************/
DeckLinkCaptureDelegate::~DeckLinkCaptureDelegate()
{
	pthread_mutex_destroy(&m_mutex);
}

/*****************************************************************************
 *	Internal semaphores, used by the decklink driver
 *****************************************************************************/
ULONG DeckLinkCaptureDelegate::AddRef(void)
{
	pthread_mutex_lock(&m_mutex);
		m_refCount++;
	pthread_mutex_unlock(&m_mutex);

	return (ULONG)m_refCount;
}

/*****************************************************************************
 *	Internal semaphores, used by the decklink driver
 *****************************************************************************/
ULONG DeckLinkCaptureDelegate::Release(void)
{
	pthread_mutex_lock(&m_mutex);
		m_refCount--;
	pthread_mutex_unlock(&m_mutex);

	if (m_refCount == 0)
	{
		delete this;
		return 0;
	}

	return (ULONG)m_refCount;
}

/*****************************************************************************
 *	limits given numbers to a range 0<n<255
 *****************************************************************************/
inline unsigned int Clamp(int n)
{
    n = n>255 ? 255 : n;
    return n<0 ? 0 : n;
}

/*****************************************************************************
 *	converts a given yuv422 Mat object to a gray one
 *****************************************************************************/
void DeckLinkCaptureDelegate::yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray)
{
  unsigned width = gray.cols;
  unsigned height = gray.rows;
  unsigned gray_skip = gray.step[0] - width;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* gray_buffer = gray.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // u y1 v y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, gray_buffer += gray_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         ++xIdx, ++gray_buffer, yuv_buffer += 2 )
    {
      *gray_buffer = yuv_buffer[1];
    }
  }
}

/*****************************************************************************
 *	converts a given yuv422 Mat object to a bgr one
 *****************************************************************************/
void DeckLinkCaptureDelegate::yuv422ToColor(const cv::Mat& yuv, cv::Mat& color)
{
  unsigned width = color.cols;
  unsigned height = color.rows;
  unsigned bgr_skip = color.step[0] - width*3;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* bgr_buffer = color.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // 0  1   2  3
  // u  y1  v  y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, bgr_buffer += bgr_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         xIdx += 2, bgr_buffer += 6, yuv_buffer += 4 )
    {
      int v = yuv_buffer[2] - 128;
      int u = yuv_buffer[0] - 128;

      bgr_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));

      bgr_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
    }
  }
}

/*****************************************************************************
 *	converts decklink internal yuv422 to a Mat object
 *****************************************************************************/
cv::Mat DeckLinkCaptureDelegate::uyvy_to_mat (int width, int height, unsigned char *src)
{
	cv::Mat image(height, width, CV_8UC3);

	int iPixelCounter = 0;
	int iLineCounter = 0;
	int iWordCounter = 0;
	int iMemCounter = 0;

	unsigned int r1,g1,b1,r2,g2,b2;

	int y1,y2,cb,cr;
	double f1,f2,f3,f4,f5,f6;

	#pragma omp parallel
	{

		#pragma omp for private (iPixelCounter,cb,cr,y1,y2,f1,f2,f3,f4,f5,f6,r1,r2,g1,g2,b1,b2, iWordCounter,iMemCounter)
		for (iLineCounter=0 ; iLineCounter < height ; iLineCounter++)
		{
			iMemCounter=(iLineCounter*width*2);

			for (iPixelCounter=0 ; iPixelCounter < width ; iPixelCounter+=2)
			{
				iWordCounter=iMemCounter+iPixelCounter*2;

				cb=src[iWordCounter]-128;
				y1=src[iWordCounter+1];
				cr=src[iWordCounter+2]-128;
				y2=src[iWordCounter+3];

				f1=1.164*(y1 - 16);
				f2=1.164*(y2 - 16);
				f3=1.793*cr;
				f4=0.534*cr;
				f5=0.213*cb;
				f6=2.115*cb;

				r1=Clamp(f1 + f3);
				g1=Clamp(f1 - f4 - f5);
				b1=Clamp(f1 + f6);

				r2=Clamp(f2 + f3);
				g2=Clamp(f2 - f4 - f5);
				b2=Clamp(f2 + f6);

				image.at<cv::Vec3b>(iLineCounter, iPixelCounter)[0]=b1;
				image.at<cv::Vec3b>(iLineCounter, iPixelCounter)[1]=g1;
				image.at<cv::Vec3b>(iLineCounter, iPixelCounter)[2]=r1;

				image.at<cv::Vec3b>(iLineCounter, iPixelCounter+1)[0]=b2;
				image.at<cv::Vec3b>(iLineCounter, iPixelCounter+1)[1]=g2;
				image.at<cv::Vec3b>(iLineCounter, iPixelCounter+1)[2]=r2;
			}
		}

	}
	return image;
}

/*****************************************************************************
 *	undistorts a Mat object on the basis of given camera parameters
 *****************************************************************************/
void DeckLinkCaptureDelegate::undistort( cv::Mat src, cv::Mat dst, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
	CV_Assert( dst.data != src.data );

	int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);

	cv::Mat_<double> A, Ar, I= cv::Mat_<double>::eye(3,3);

	cameraMatrix.convertTo(A, CV_64F);
	if( distCoeffs.data )
		distCoeffs = cv::Mat_<double>(distCoeffs);
	else
	{
		distCoeffs.create(5, 1, CV_64F);
		distCoeffs = 0.;
	}

	A.copyTo(Ar);

	double v0 = Ar(1, 2);

	int y=0;

	int initialstart=1;

	#pragma omp parallel
	{

		#pragma omp for private (Ar) firstprivate (initialstart)
		for( y = 0; y < src.rows; y += stripe_size0 )
		{
			int stripe_size = std::min( stripe_size0, src.rows - y );

			cv::Mat dst_part = dst.rowRange(y, y + stripe_size);

			if (initialstart==1)
			{
				Ar=A.clone();
				initialstart=0;
			}

			Ar(1, 2) = v0 - y;

			cv::Mat map1_part, map2_part;

			cv::Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);

			map1_part = map1.rowRange(0, stripe_size);
			map2_part = map2.rowRange(0, stripe_size);


			initUndistortRectifyMap( A, distCoeffs, I, Ar, cv::Size(src.cols, stripe_size),map1_part.type(), map1_part, map2_part );

			remap( src, dst_part, map1_part, map2_part, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
		}
	}
}

/*****************************************************************************
 *	This function is called everytime a video frame is ready to be captured
 *****************************************************************************/
HRESULT DeckLinkCaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioFrame)
{
	IDeckLinkVideoFrame*	                rightEyeFrame = NULL;
	IDeckLinkVideoFrame3DExtensions*        threeDExtensions = NULL;
	void*									frameBytes;

	ros::Time time = ros::Time::now();

	cv::Mat	mOutput,mCopy,mTest;

	if(videoFrame)
	{
		// If 3D mode is enabled we retreive the 3D extensions interface which gives.
		// us access to the right eye frame by calling GetFrameForRightEye() .
		if ( (videoFrame->QueryInterface(IID_IDeckLinkVideoFrame3DExtensions, (void **) &threeDExtensions) != S_OK) || (threeDExtensions->GetFrameForRightEye(&rightEyeFrame) != S_OK))
		{
			rightEyeFrame = NULL;
		}

		if (threeDExtensions)
		{
			threeDExtensions->Release();
		}

		if (videoFrame->GetFlags() & bmdFrameHasNoInputSource)
		{
			ROS_ERROR("Frame received (%i) - No input signal detected",frameCount);
		}
		else
		{
			// if image rate is reduced by dynamic_reconfigure(var ratereduction) the image might be dropped here
			if (imagedrop < ::ratereduction)
			{
				imagedrop++;
			}
			else
			{

				const char *timecodeString = NULL;
				if (g_timecodeFormat != 0)
				{
					IDeckLinkTimecode *timecode;
					if (videoFrame->GetTimecode(g_timecodeFormat, &timecode) == S_OK)
					{
						timecode->GetString(&timecodeString);
					}
				}
				if (timecodeString)
				{
					free((void*)timecodeString);
				}

				// Take the Video here...
				videoFrame->GetBytes(&frameBytes);

				int width=videoFrame->GetWidth();
				int height=videoFrame->GetHeight();


				mOutput=this->uyvy_to_mat(width, height, (unsigned char*) frameBytes);

				if (::resolution==1)
				{
					mCopy=mOutput.clone();

					cv::resize(mOutput , mCopy, cv::Size(1280,720) , 0.0, 0.0, cv::INTER_LINEAR);
				}
				else
				{
					mCopy=mOutput;
				}

				camera_info.width=mCopy.cols;
				camera_info.height=mCopy.rows;

				// convert OpenCV image to ROS message
				cv_bridge::CvImage cvi;
				cvi.header.stamp = time;
				cvi.header.frame_id = "camera";
				cvi.encoding = "bgr8";

				//only publish distorted image if any node is asking for it
				if (pub_distorted.getNumSubscribers()>0)
				{

					cvi.image = mCopy;
					sensor_msgs::Image im_distorted;
					cvi.toImageMsg(im_distorted);

					pub_distorted.publish(im_distorted, camera_info);
				}

				//only publish undistorted image if any node is asking for it
				if (pub.getNumSubscribers()>0)
				{
					cv::Mat mOutputUndistort(mCopy.rows,mCopy.cols, CV_8UC3);

					cv::Mat k;
					k.create(3, 3, CV_32FC1);

					cv::Mat d;
					d.create(1,5,CV_32FC1);
														// Values for USIEGEN Full HD camera
					k.at<float>(0,0)=camera_info.K[0]; 	//1388.242699;
					k.at<float>(0,1)=camera_info.K[1];	//0.000000;
					k.at<float>(0,2)=camera_info.K[2];	//1060.095445;
					k.at<float>(1,0)=camera_info.K[3];	//0.000001;
					k.at<float>(1,1)=camera_info.K[4];	//1359.502461;
					k.at<float>(1,2)=camera_info.K[5];	//477.621371;
					k.at<float>(2,0)=camera_info.K[6];	//0.000001;
					k.at<float>(2,1)=camera_info.K[7];	//0.000001;
					k.at<float>(2,2)=camera_info.K[8];	//1.000000;

					d.at<float>(0)=camera_info.D[0];	//-0.157918;
					d.at<float>(1)=camera_info.D[1];	//0.117873;
						d.at<float>(2)=camera_info.D[2];	//-0.001324;
					d.at<float>(3)=camera_info.D[3];	//0.000294;
					d.at<float>(4)=camera_info.D[4];	//0.000000;

					this->undistort(mCopy,mOutputUndistort,k,d);


 	
/*					Save images to disk
 					vector<int> compression_params;
    					compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    					compression_params.push_back(9);

					counter=counter+1;
					
					string base_name="//positives//positive_";
					stringstream convert;
					convert << counter;
					string extension_name=".png"; 


    					try 
					{
        					imwrite(base_name+convert.str()+extension_name, mOutputUndistort, compression_params);
    					}
				    	catch (runtime_error& ex) 
					{
        					cout << "ERROR writing to disk" << endl;
        					return 1;
    					}

    					cout << "Saved PNG file with alpha data." << endl;

*/



					cvi.image = mOutputUndistort;

					sensor_msgs::Image im;
					cvi.toImageMsg(im);

					pub.publish(im, camera_info);
				}
				imagedrop=1;
			}
		}

		if (rightEyeFrame)
		{
			rightEyeFrame->Release();
		}

		frameCount++;
	}
	return S_OK;
}


/*****************************************************************************
 *	We do not intend to switch the video format during runtime.
 *	That's why this section is empty at the moment - it can be extended if
 *	necessary.
 *****************************************************************************/
HRESULT DeckLinkCaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode, BMDDetectedVideoInputFormatFlags)
{
    return S_OK;
}

