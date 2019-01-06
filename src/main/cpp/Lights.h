#ifndef SRC_LIGHTS_H_
#define SRC_LIGHTS_H_

#include "Intake.h"
#include "Scaler.h"
#include "Drive.h"
#include "Controls.h"
#include "Serialport.h"
#include "Robotbase.h"

using namespace frc;

class Lights : public RobotBase {

public:
	Lights(Intake&, Scaler&, Drive&, Controls&);
	~Lights();

	void initialize();
	void process();

private:
	Intake& intake;
	Scaler& scaler;
	Drive& drive;
	Controls& controls;
	SerialPort Serial1;
	std::thread serialWriter;
	char output[2];
	static void callWriteToStream(uint32_t arg);
	void writeToStream();
};
#endif /* SRC_LIGHTS_H_ */

