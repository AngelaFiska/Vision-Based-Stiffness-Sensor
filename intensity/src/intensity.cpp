/*
 * intensity.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: mende
 */

#include "intensity.h"
#include "capture.h"
#include "DeckLinkAPIDispatch.cpp"
#include <sensor_msgs/CameraInfo.h>
#include "camera_calibration_parsers/parse_ini.h"
#include <ros/package.h>

const BMDPixelFormat	gKnownPixelFormats[]		= {bmdFormat8BitYUV, bmdFormat10BitYUV, bmdFormat8BitARGB, bmdFormat8BitBGRA, bmdFormat10BitRGB, 0};
const char *			gKnownPixelFormatNames[]	= {" 8-bit YUV", "10-bit YUV", "8-bit ARGB", "8-bit BGRA", "10-bit RGB", NULL};

extern sensor_msgs::CameraInfo camera_info;

extern int ratereduction;
extern int resolution;
extern std::string intensity_path;

/*****************************************************************************
 *	Constructor
 *****************************************************************************/
intensity_class::intensity_class(ros::NodeHandle intensity_nh)
{
	// Create an IDeckLinkIterator object to enumerate all DeckLink cards in the system
	deckLinkIterator = ::CreateDeckLinkIteratorInstance();

	numDevices=0;
	g_timecodeFormat = 0;
	g_videoModeIndex = -1;
	g_audioChannels = 2;
	g_audioSampleDepth = 16;
	g_audioOutputFile = NULL;
	g_maxFrames = -1;

	frameCount = 0;
	displayModeCount=0;
	videoOutputFile = -1;

	selectedDisplayMode = bmdModeNTSC;
	pixelFormat = bmdFormat8BitYUV;
	foundDisplayMode = false;

	inputFlags = 0;
}


/*****************************************************************************
 *	Destructor
 *****************************************************************************/
intensity_class::~intensity_class()
{

}


/*****************************************************************************
 *	This function initializes decklink frame grabber and checks necessary
 *	prerequisites
 *****************************************************************************/
bool intensity_class::StartStreaming ()
{
		::intensity_path = ros::package::getPath("intensity");

		deckLinkIterator = ::CreateDeckLinkIteratorInstance();

		exitStatus = false;

		if (!deckLinkIterator)
		{
			ROS_ERROR("This application requires the DeckLink drivers installed.");
			return false;
		}

		/* Connect to the first DeckLink instance */
		result = deckLinkIterator->Next(&deckLink);
		if (result != S_OK)
		{
			ROS_ERROR("No DeckLink PCI cards found.");
			return false;
		}

		if (deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput) != S_OK)
		{
			return false;
		}

		// Define callback function for incoming frames
		delegate = new DeckLinkCaptureDelegate();
		deckLinkInput->SetCallback(delegate);

		if (::resolution==1)
		{
			if (camera_calibration_parsers::readCalibrationIni(::intensity_path+"/camera_parameters_HDR.txt", camera_name, /*delegate->*/::camera_info))
				{
					ROS_INFO("Successfully read camera calibration HDR.  Rerun camera calibrator if it is incorrect.");
				}
				else
				{
					ROS_ERROR("No camera_parameters_HDR.txt file found.  Use default file if no other is available.");
				}
		}
		else
		{
			if (camera_calibration_parsers::readCalibrationIni(::intensity_path+"/camera_parameters_HD.txt", camera_name, /*delegate->*/::camera_info))
				{
					ROS_INFO("Successfully read camera calibration HD.  Rerun camera calibrator if it is incorrect.");
				}
				else
				{
					ROS_ERROR("No camera_parameters_HD.txt file found.  Use default file if no other is available.");
				}
		}

		//new callbak function needs to get information about the publishers
		delegate->pub=this->pub;
		delegate->pub_distorted=this->pub_distorted;

		// Obtain an IDeckLinkDisplayModeIterator to enumerate the display modes supported on output
		result = deckLinkInput->GetDisplayModeIterator(&displayModeIterator);
		if (result != S_OK)
		{
			ROS_ERROR("Could not obtain the video output display mode iterator - result = %08x");
			return false;
		}

		pixelFormat = bmdFormat8BitYUV;
		g_videoModeIndex=10;  //or 11?


		if (g_videoModeIndex < 0)
		{
			ROS_ERROR("No video mode specified");
			usage(0);
		}

		//Select correct video mode
		while (displayModeIterator->Next(&displayMode) == S_OK)
		{
			if (g_videoModeIndex == displayModeCount)
			{
				BMDDisplayModeSupport result;
				const char *displayModeName;

				foundDisplayMode = true;
				displayMode->GetName(&displayModeName);
				selectedDisplayMode = displayMode->GetDisplayMode();

				deckLinkInput->DoesSupportVideoMode(selectedDisplayMode, pixelFormat, bmdVideoInputFlagDefault, &result, NULL);

				if (result == bmdDisplayModeNotSupported)
				{
					ROS_ERROR("The display mode %s is not supported with the selected pixel format", displayModeName);
					return false;
				}

				if (inputFlags & bmdVideoInputDualStream3D)
				{
					if (!(displayMode->GetFlags() & bmdDisplayModeSupports3D))
					{
						ROS_ERROR("The display mode %s is not supported with 3D", displayModeName);
						return false;
					}
				}

				break;
			}
			displayModeCount++;
			displayMode->Release();
		}

		// We should have a valid video mode at this stage...
		if (!foundDisplayMode)
		{
			ROS_ERROR("Invalid mode %d specified", g_videoModeIndex);
			return false;
		}

		//Enable video capture!
	    result = deckLinkInput->EnableVideoInput(selectedDisplayMode, pixelFormat, inputFlags);
	    if(result != S_OK)
	    {
			ROS_ERROR("Failed to enable video input. Is another application using the card?");
			return false;
	    }

	    //Start streaming!
		result = deckLinkInput->StartStreams();
	    if(result != S_OK)
	    {
			return false;
	    }

		return true;
}


/*****************************************************************************
 *	Cleans up after the job is done.
 *****************************************************************************/
bool intensity_class::StopStreaming ()
{
		// This procedure is only for cleaning up...
		if (displayModeIterator != NULL)
		{
			displayModeIterator->Release();
			displayModeIterator = NULL;
		}

		if (deckLinkInput != NULL)
		{
			deckLinkInput->Release();
			deckLinkInput = NULL;
		}

		if (deckLink != NULL)
		{
			deckLink->Release();
			deckLink = NULL;
		}

		if (deckLinkIterator != NULL)
			deckLinkIterator->Release();

		return exitStatus;

	return true;
}


/*****************************************************************************
 *	Checks if a suitable frame grabber card is installed
 *****************************************************************************/
int	intensity_class::CheckDevices ()
{

	if (deckLinkIterator == NULL)
	{
		ROS_ERROR("A DeckLink iterator could not be created.  The DeckLink drivers may not be installed.");
		return false;
	}

	// Enumerate all cards in this system
	while (deckLinkIterator->Next(&deckLink) == S_OK)
	{
		char *		deviceNameString = NULL;

		// Increment the total number of DeckLink cards found
		numDevices++;
		if (numDevices > 1)

		// *** Print the model name of the DeckLink card
		result = deckLink->GetModelName((const char **) &deviceNameString);
		if (result == S_OK)
		{
			ROS_INFO("=============== %s ===============", deviceNameString);
			free(deviceNameString);
		}

		print_attributes(deckLink);

		// ** List the video output display modes supported by the card
		print_output_modes(deckLink);

		// ** List the input and output capabilities of the card
		print_capabilities(deckLink);

		// Release the IDeckLink instance when we've finished with it to prevent leaks
		deckLink->Release();
	}

	deckLinkIterator->Release();

	// If no DeckLink cards were found in the system, inform the user
	if (numDevices == 0)
	{
		ROS_ERROR("No Blackmagic Design devices were found.");
		return false;
	}

	return true;
}


/*****************************************************************************
 *	Prints all attributes of the installed card(s) - for debugging purposes
 *****************************************************************************/
void intensity_class::print_attributes (IDeckLink* deckLink)
{
	IDeckLinkAttributes*				deckLinkAttributes = NULL;
	bool								supported;
	int64_t								count;
	char *								serialPortName = NULL;
	HRESULT								result;

	// Query the DeckLink for its attributes interface
	result = deckLink->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes);
	if (result != S_OK)
	{
		ROS_ERROR("Could not obtain the IDeckLinkAttributes interface - result = %08x\n", result);
		goto bail;
	}

	// List attributes and their value
	printf("Attribute list:\n");

	result = deckLinkAttributes->GetFlag(BMDDeckLinkHasSerialPort, &supported);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %s", "Serial port present ?", (supported == true) ? "Yes" : "No");

		if (supported)
		{
			result = deckLinkAttributes->GetString(BMDDeckLinkSerialPortDeviceName, (const char **) &serialPortName);
			if (result == S_OK)
			{
				ROS_INFO(" %-40s %s", "Serial port name: ", serialPortName);
				free(serialPortName);
			}
			else
			{
				ROS_INFO("Could not query the serial port presence attribute- result = %08x", result);
			}
		}

	}
	else
	{
		ROS_INFO("Could not query the serial port presence attribute- result = %08x", result);
	}

    result = deckLinkAttributes->GetInt(BMDDeckLinkNumberOfSubDevices, &count);
    if (result == S_OK)
    {
        ROS_INFO(" %-40s %lld", "Number of sub-devices:",  count);
        if (count != 0)
        {
            result = deckLinkAttributes->GetInt(BMDDeckLinkSubDeviceIndex, &count);
            if (result == S_OK)
            {
                ROS_INFO(" %-40s %lld", "Sub-device index:",  count);
            }
            else
            {
                ROS_INFO("Could not query the sub-device index attribute- result = %08x", result);
            }
        }
    }
    else
    {
        ROS_INFO("Could not query the number of sub-device attribute- result = %08x", result);
    }

	result = deckLinkAttributes->GetInt(BMDDeckLinkMaximumAudioChannels, &count);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %lld", "Number of audio channels:",  count);
	}
	else
	{
		ROS_ERROR("Could not query the number of supported audio channels attribute- result = %08x", result);
	}

	result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &supported);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %s", "Input mode detection supported ?", (supported == true) ? "Yes" : "No");
	}
	else
	{
		ROS_ERROR("Could not query the input mode detection attribute- result = %08x", result);
	}

	result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInternalKeying, &supported);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %s", "Internal keying supported ?", (supported == true) ? "Yes" : "No");
	}
	else
	{
		ROS_ERROR("Could not query the internal keying attribute- result = %08x", result);
	}

	result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsExternalKeying, &supported);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %s", "External keying supported ?", (supported == true) ? "Yes" : "No");
	}
	else
	{
		ROS_ERROR("Could not query the external keying attribute- result = %08x", result);
	}

	result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsHDKeying, &supported);
	if (result == S_OK)
	{
		ROS_INFO(" %-40s %s", "HD-mode keying supported ?", (supported == true) ? "Yes" : "No");
	}
	else
	{
		ROS_ERROR("Could not query the HD-mode keying attribute- result = %08x", result);
	}

bail:
	if(deckLinkAttributes != NULL)
		deckLinkAttributes->Release();

}


/*****************************************************************************
 *	Prints output modes of the installed card(s) - for debugging purposes
 *****************************************************************************/
void intensity_class::print_output_modes (IDeckLink* deckLink)
{
	IDeckLinkOutput*					deckLinkOutput = NULL;
	IDeckLinkDisplayModeIterator*		displayModeIterator = NULL;
	IDeckLinkDisplayMode*				displayMode = NULL;
	HRESULT								result;

	// Query the DeckLink for its configuration interface
	result = deckLink->QueryInterface(IID_IDeckLinkOutput, (void**)&deckLinkOutput);
	if (result != S_OK)
	{
		ROS_ERROR("Could not obtain the IDeckLinkOutput interface - result = %08x", result);
		goto bail;
	}

	// Obtain an IDeckLinkDisplayModeIterator to enumerate the display modes supported on output
	result = deckLinkOutput->GetDisplayModeIterator(&displayModeIterator);
	if (result != S_OK)
	{
		ROS_ERROR("Could not obtain the video output display mode iterator - result = %08x", result);
		goto bail;
	}

	// List all supported output display modes
	ROS_INFO("Supported video output display modes and pixel formats:");
	while (displayModeIterator->Next(&displayMode) == S_OK)
	{
		char *			displayModeString = NULL;

		result = displayMode->GetName((const char **) &displayModeString);
		if (result == S_OK)
		{
			char					modeName[64];
			int						modeWidth;
			int						modeHeight;
			BMDTimeValue			frameRateDuration;
			BMDTimeScale			frameRateScale;
			int						pixelFormatIndex = 0; // index into the gKnownPixelFormats / gKnownFormatNames arrays
			BMDDisplayModeSupport	displayModeSupport;


			// Obtain the display mode's properties
			modeWidth = displayMode->GetWidth();
			modeHeight = displayMode->GetHeight();
			displayMode->GetFrameRate(&frameRateDuration, &frameRateScale);
			ROS_INFO(" %-20s \t %d x %d \t %7g FPS\t", displayModeString, modeWidth, modeHeight, (double)frameRateScale / (double)frameRateDuration);

			// Print the supported pixel formats for this display mode
			while ((gKnownPixelFormats[pixelFormatIndex] != 0) && (gKnownPixelFormatNames[pixelFormatIndex] != NULL))
			{
				if ((deckLinkOutput->DoesSupportVideoMode(displayMode->GetDisplayMode(), gKnownPixelFormats[pixelFormatIndex], bmdVideoOutputFlagDefault, &displayModeSupport, NULL) == S_OK)
						&& (displayModeSupport != bmdDisplayModeNotSupported))
				{
					ROS_INFO("%s\t", gKnownPixelFormatNames[pixelFormatIndex]);
				}
				pixelFormatIndex++;
			}
			free(displayModeString);
		}

		// Release the IDeckLinkDisplayMode object to prevent a leak
		displayMode->Release();
	}

	printf("\n");

bail:
	// Ensure that the interfaces we obtained are released to prevent a memory leak
	if (displayModeIterator != NULL)
		displayModeIterator->Release();

	if (deckLinkOutput != NULL)
		deckLinkOutput->Release();
}


/*****************************************************************************
 *	Prints capabilities of the installed card(s) - for debugging purposes
 *****************************************************************************/
void intensity_class::print_capabilities (IDeckLink* deckLink)
{
	IDeckLinkAttributes*		deckLinkAttributes = NULL;
	int64_t						ports;
	int							itemCount;
	HRESULT						result;

	// Query the DeckLink for its configuration interface
	result = deckLink->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes);
	if (result != S_OK)
	{
		ROS_ERROR("Could not obtain the IDeckLinkAttributes interface - result = %08x", result);
		goto bail;
	}

	ROS_INFO("Supported video output connections:  ");
	itemCount = 0;
	result = deckLinkAttributes->GetInt(BMDDeckLinkVideoOutputConnections, &ports);
	if (result == S_OK)
	{
		if (ports & bmdVideoConnectionSDI)
		{
			itemCount++;
			ROS_INFO("SDI");
		}

		if (ports & bmdVideoConnectionHDMI)
		{
			if (itemCount++ > 0)
			ROS_INFO("HDMI");
		}

		if (ports & bmdVideoConnectionOpticalSDI)
		{
			if (itemCount++ > 0)
			ROS_INFO("Optical SDI");
		}

		if (ports & bmdVideoConnectionComponent)
		{
			if (itemCount++ > 0)
			ROS_INFO("Component");
		}

		if (ports & bmdVideoConnectionComposite)
		{
			if (itemCount++ > 0)
			ROS_INFO("Composite");
		}

		if (ports & bmdVideoConnectionSVideo)
		{
			if (itemCount++ > 0)
			ROS_INFO("S-Video");
		}
	}
	else
	{
		ROS_ERROR("Could not obtain the list of output ports - result = %08x", result);
		goto bail;
	}


	ROS_INFO("Supported video input connections:");
	itemCount = 0;
	result = deckLinkAttributes->GetInt(BMDDeckLinkVideoInputConnections, &ports);
	if (result == S_OK)
	{
		if (ports & bmdVideoConnectionSDI)
		{
			itemCount++;
			ROS_INFO("SDI");
		}

		if (ports & bmdVideoConnectionHDMI)
		{
			if (itemCount++ > 0)
			ROS_INFO("HDMI");
		}

		if (ports & bmdVideoConnectionOpticalSDI)
		{
			if (itemCount++ > 0)
			ROS_INFO("Optical SDI");
		}

		if (ports & bmdVideoConnectionComponent)
		{
			if (itemCount++ > 0)
			ROS_INFO("Component");
		}

		if (ports & bmdVideoConnectionComposite)
		{
			if (itemCount++ > 0)
			ROS_INFO("Composite");
		}

		if (ports & bmdVideoConnectionSVideo)
		{
			if (itemCount++ > 0)
			ROS_INFO("S-Video");
		}
	}
	else
	{
		ROS_ERROR("Could not obtain the list of input ports - result = %08x", result);
		goto bail;
	}

bail:
	if (deckLinkAttributes != NULL)
		deckLinkAttributes->Release();
}



/*****************************************************************************
 *	Gives hints how to use the card if given parameters are not suitable
 *****************************************************************************/
int intensity_class::usage(int status)
{
	HRESULT result;
	IDeckLinkDisplayMode *displayMode;
	int displayModeCount = 0;

	ROS_ERROR("Usage: Capture -m <mode id> [OPTIONS] -m <mode id>"
	);

    while (displayModeIterator->Next(&displayMode) == S_OK)
    {
        char *          displayModeString = NULL;

        result = displayMode->GetName((const char **) &displayModeString);
        if (result == S_OK)
        {
			BMDTimeValue frameRateDuration, frameRateScale;
            displayMode->GetFrameRate(&frameRateDuration, &frameRateScale);

			ROS_ERROR("        %2d:  %-20s \t %li x %li \t %g FPS",
				displayModeCount, displayModeString, displayMode->GetWidth(), displayMode->GetHeight(), (double)frameRateScale / (double)frameRateDuration);

            free(displayModeString);
			displayModeCount++;
        }

        // Release the IDeckLinkDisplayMode object to prevent a leak
        displayMode->Release();
    }

	exit(status);
}


/*****************************************************************************
 *	callback function for camera_calibration toolkit. The received
 *	information is stored in camera parameter files - one for each
 *	available resolution.
 *****************************************************************************/
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp)
{
   ROS_INFO("New camera info received");

   if (::resolution==1) //resolution=HD ready write calibration file
   {
	   camera_info = req.camera_info;

	   if (camera_calibration_parsers::writeCalibrationIni(::intensity_path+"/camera_parameters_HDR.txt", "EndoCam 5509", camera_info))
	  {
	   ROS_INFO("Camera information written to camera_parameters_HDR.txt");
	   return true;
	  }
	  else
	  {
	   ROS_ERROR("Could not write camera_parameters_HDR.txt");
	   return false;
	  }
   }
   else //resolution=Full HD write calibration file
   {
	   camera_info = req.camera_info;
	   if (camera_calibration_parsers::writeCalibrationIni(::intensity_path+"/camera_parameters_HD.txt", "EndoCam 5509", camera_info))
	   {
		   ROS_INFO("Camera information written to camera_parameters_HD.txt");
		   return true;
	   }
	   else
	   {
		   ROS_ERROR("Could not write camera_parameters_HD.txt");
		   return false;
	   }
   }




 }


/*****************************************************************************
 *	callback function for dynamic reconfigure toolkit. "Rate reduction" and
 *	resolution may be switched. If the resolution has changed the
 *	calibration data is updated from the stored calibration information file.
 *****************************************************************************/
void intensity_class::configCallback(intensity::intensity_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	::ratereduction = config.ratereduction;
	::resolution = config.resolution;

	::intensity_path = ros::package::getPath("intensity");

	if (::resolution==1)
	{
		if (camera_calibration_parsers::readCalibrationIni(::intensity_path+"/camera_parameters_HDR.txt", camera_name, /*delegate->*/::camera_info))
		{
			ROS_INFO("Successfully read camera calibration HDR.  Rerun camera calibrator if it is incorrect.");
		}
		else
		{
			ROS_ERROR("No camera_parameters_HDR.txt file found.  Use default file if no other is available.");
		}
	}
	else
	{
		if (camera_calibration_parsers::readCalibrationIni(::intensity_path+"/camera_parameters_HD.txt", camera_name, /*delegate->*/::camera_info))
		{
			ROS_INFO("Successfully read camera calibration HD.  Rerun camera calibrator if it is incorrect.");
		}
		else
		{
			ROS_ERROR("No camera_parameters_HD.txt file found.  Use default file if no other is available.");
		}
	}
} // end configCallback()
