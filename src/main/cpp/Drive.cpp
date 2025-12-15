#include "Drive.h"

	Drive::Drive()
		// leftMotors
		// rightMotors,
		// leftEncoder,
		// rightEncoder,
		// _leftPowers,
		// _rightPowers,
		// // _maxSpeed(0),
		// // _distance(0),
		// // _angle(0),
		// _prevRight,
		// _prevLeft,
		// turnLeft(false),

	{
		leftMotors.SetInverted(true);
	}
float Drive::RampPower(double previousPower, double wantedPower){
	if (previousPower < wantedPower -.1){
		return previousPower += .5;
	}else{
		if (previousPower > wantedPower +.1){
			return previousPower -= .5;
		}else{
			return wantedPower;
		}
	}
}
void Drive::Move_Speed(float leftSpeed, float rightSpeed) {
	_leftPowers =leftSpeed;
	_rightPowers = rightSpeed;
}

void Drive::Reset(){
	leftMotors.Set(0);
	rightMotors.Set(0);
}
void Drive::Process(){
	leftMotors.Set(_leftPowers);
	rightMotors.Set(_rightPowers);
	_prevRight = _rightPowers;
	_prevLeft = _leftPowers;
}