#ifndef FEEDBACK_H_
#define FEEDBACK_H_
#include "cscore.h"
#include <vector>
class Feedback{

public:
	Feedback();
	void initCameras();

	void send_Debug_String(const char *debugName, const char *data, const char *format,...);

	void send_Debug_Double(const char *debugName, const char *data, double format);

	void swapCamera();
	bool isFrontCamera();

private:
	cs::MjpegServer *camServer;
	std::vector<cs::UsbCamera *> camVec;
	bool frontCam;
	bool hasBackCam;

};


#endif /* FEEDBACK_H_ */
