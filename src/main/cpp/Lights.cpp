#include "Lights.h"

Lights::Lights(Intake& i, Scaler& s, Drive& d, Controls& c) :
				intake(i),
				scaler(s),
				drive(d),
				controls(c),
				Serial1(9600, SerialPort::kMXP)


{
		//serialWriter("Lights Writer Thread",Lights::callWriteToStream, this);
}
Lights::~Lights() {

}
void Lights::initialize() {
	Serial1.Reset();
}
void Lights::callWriteToStream(uint32_t arg) {
	((Lights*) arg)->writeToStream();
}

void Lights::writeToStream() {
	while (true) {
		char bytes[2];
		{
			bytes[0] = output[0];
			bytes[1] = output[1];
		}
		printf("%x %x\n", bytes[0], bytes[1]);
		Serial1.Write(bytes, 2);
		Serial1.Flush();
		Wait(0.050);
	}
}
void Lights::process() {
bool TeleopEnabled = RobotBase::IsOperatorControl();

	char Bytes[2];
	Bytes[0] = 0;
	Bytes[1] = 1;

if (TeleopEnabled) {
		Bytes[1] = Bytes[1] | 0x02;
	}
	{
	output[0] = Bytes[0];
	output[1] = Bytes[1];
	}
}


