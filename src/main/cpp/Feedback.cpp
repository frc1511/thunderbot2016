#include "Feedback.h"
#include "frc/WPILib.h"
using namespace frc;
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include <cscore_cpp.h>
#include <cscore_oo.h>
#include <cscore_oo.inl>
using namespace cs;

static bool camTwoIsPresent()
{
	std::vector<UsbCameraInfo>  camInfos = UsbCamera::EnumerateUsbCameras();
	for (unsigned i = 0; i < camInfos.size(); ++i) {
		printf("Found camera #%d named %s at path %s\n", camInfos[i].dev, camInfos[i].name.c_str(), camInfos[i].path.c_str());
		if (camInfos[i].dev == 1)
			return true;
	}
	return false;
}

static bool camOneIsPresent()
{
	std::vector<UsbCameraInfo>  camInfos = UsbCamera::EnumerateUsbCameras();
	for (unsigned i = 0; i < camInfos.size(); ++i) {
		printf("Found camera #%d named %s at path %s\n", camInfos[i].dev, camInfos[i].name.c_str(), camInfos[i].path.c_str());
		if (camInfos[i].dev == 0)
			return true;
	}

	return false;
}



	Feedback::Feedback() : camServer(NULL), frontCam(true), hasBackCam(false)

	{
		camServer = new MjpegServer("server", 8888);
	}

	void Feedback::initCameras() {
		std::vector<UsbCameraInfo>  camInfos = UsbCamera::EnumerateUsbCameras();


		std::vector<std::string> v;

		for (int i = 0; i < camInfos.size(); ++i) {
			printf("Found camera #%d named %s at path %s\n", camInfos[i].dev, camInfos[i].name.c_str(), camInfos[i].path.c_str());
			if (camInfos[i].dev == 0) {
				v.push_back(camInfos[i].path);
	#ifdef USE_TWO_CAMERAS
			} else if (camInfos[i].dev == 1) {
				v.push_back(camInfos[i].path);
				hasBackCam = true;
	#endif
			}
		}


		// defaults seem to be: Auto white bal; exposure 50%; brigthness 80%
		//
		// axis was: color level 70; brightness 50; sharpness 0; white balance Fixed Fluorescent 1; exposure Auto; no exposure priority
		for (size_t i = 0; i < v.size(); ++i) {
			UsbCamera *camera =	new UsbCamera(i == 0 ? "cam0" : "cam1", v[i]);

			camera->SetExposureAuto();
			camera->SetBrightness(50);
			camera->SetWhiteBalanceManual(VideoCamera::kFixedFluorescent1);
			camVec.push_back(camera);
		}
		camServer->SetSource(*(camVec[0]));

	}


void Feedback::send_Debug_String(const char *subsystemName, const char *dataName, const char *format, ...) {//

	std::string keyName;
		keyName = subsystemName;
		keyName += "_";
		keyName += dataName; //adds a _ after your subsytem name, the name of your specific value after that
	  char buffer[256]; //sets the maximum character count to 256
	  va_list args; // args is the "..."
	  va_start (args, format); //marks the start of the list of arguments(args)
	  vsnprintf (buffer,256,format, args); //printing the args after the keyName
	  va_end (args);//marks end of arguments (this allows us to change the arguments and not the keyname)
	  SmartDashboard::PutString(keyName, buffer);//this prints keyName into the table, then the arguments
	}

void Feedback::send_Debug_Double(const char *subsystemName, const char *dataName, double format){

	std::string keyName;
	keyName = subsystemName;
	keyName += "_";
	keyName += dataName; // a _ after the subsystem name, and adds the name of the value after that
	SmartDashboard::PutNumber(keyName, format); //this prints keyName and the data into the table
}

void Feedback::swapCamera()
{
#ifdef USE_TWO_CAMERAS
	if (hasBackCam) {
		UsbCamera *cam = camVec[frontCam ? 0 : 1];

        frontCam = !frontCam;
        camServer->SetSource(*cam);
	}
#endif
}

bool Feedback::isFrontCamera()
{
	return (frontCam);
}

