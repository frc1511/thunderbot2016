#include "frc/WPILib.h"
#include "Drive.h"
#include "IOMap.h"

#define DRIVE_AUTO_ANGLE_TOLERANCE 4.0f
#define DRIVE_AUTO_DEBUG 0

#ifdef robot_2
const double DRIVE_AUTO_LEFT_SCALE = 1.0;
const double DRIVE_AUTO_RIGHT_SCALE = .95;
#else
const double DRIVE_AUTO_LEFT_SCALE = 1.0;
const double DRIVE_AUTO_RIGHT_SCALE = 1.0;
#endif



	Drive::Drive() :
		leftMotors(PWM_OUT_DRIVE_LEFT),
		rightMotors(PWM_OUT_DRIVE_RIGHT),
		leftEncoder(DIG_IO_DRIVE_SHAFT_ENCODER_LEFT_A, DIG_IO_DRIVE_SHAFT_ENCODER_LEFT_B),
		rightEncoder(DIG_IO_DRIVE_SHAFT_ENCODER_RIGHT_A, DIG_IO_DRIVE_SHAFT_ENCODER_RIGHT_B),
		gyro(ANALOG_IN_DRIVE_GYRO),
		calibrate(DIG_IO_GYRO_CALIBRATE),
		_leftPowers(0),
		_rightPowers(0),
		_maxSpeed(0),
		_distance(0),
		_angle(0),
		_prevRight(0),
		_prevLeft(0),
		initGyro(0),
		_gyroCalibrated(false),
		turnLeft(false),
		drive(kDisabled)

	{
		leftMotors.SetInverted(true);
		mathAngle = 0;
		powerTurn = 0;
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
	drive = kTeleop;
}
void Drive::Auto_Straight(float maxSpeed, float distance) {
	_distance = distance * 48;
	_rightPowers = maxSpeed;
	_leftPowers = maxSpeed;
	_maxSpeed = maxSpeed;
	leftEncoder.Reset();
	rightEncoder.Reset();
	initGyro = gyro.GetAngle();
	drive = kAutoStraight;
}
void Drive::Auto_Turn(float angle){
#if DRIVE_AUTO_DEBUG
	printf("the great turn is underway \n");
#endif
	_angle = angle;
	powerTurn = 0;
	mathAngle = gyro.GetAngle();
	turnLeft = false;
	drive = kAutoTurn;
}
void Drive::Auto_Turn_Moat(float angle){
#if DRIVE_AUTO_DEBUG
	printf("the great turn is underway \n");
#endif
	_angle = angle;
	powerTurn = 0;
	mathAngle = gyro.GetAngle();
	turnLeft = true;
	drive = kAutoTurn;
}
bool Drive::Auto_Move_Complete(){
	switch(drive) {
		case kAutoStraight:
			return false;
		case kAutoTurn:
			return false;
		case kAuto:
			return true;
		default:
			return true;
	}
}

void Drive::Debug(Feedback *feedback){
	feedback->send_Debug_Double("Drive","leftEncoder",(double)leftEncoder.Get());
	feedback->send_Debug_Double("Drive","rightEncoder",(double)rightEncoder.Get());
	feedback->send_Debug_Double("Drive","Gyro",(double)gyro.GetAngle());
}
void Drive::Reset(){
	leftMotors.Set(0);
	rightMotors.Set(0);
	gyro.Reset();
}
void Drive::Process(){
	switch(drive) {
	case kAutoStraight:
	//	if (initGyro > gyro.GetAngle() && _distance > (leftEncoder.Get() + rightEncoder.Get())/2){
		//	_leftPowers *= 1.2;
			//_rightPowers *= .8;
		//}
		//if (initGyro < gyro.GetAngle() && _distance > (leftEncoder.Get() + rightEncoder.Get())/2){
			//_rightPowers *= 1.2;
			//_leftPowers *= .8;
		//}
		//if (initGyro == gyro.GetAngle() && _distance > (leftEncoder.Get() + rightEncoder.Get())/2){
			//_rightPowers = _maxSpeed;
			//_leftPowers = _maxSpeed;
		//}
		if (_distance <= (rightEncoder.Get())){//(-leftEncoder.Get() + rightEncoder.Get())/2){
			_rightPowers = 0;
			_leftPowers = 0;
			drive = kAuto;
		}
		break;
	case kAutoTurn:
		mathAngle = gyro.GetAngle();
#if DRIVE_AUTO_DEBUG
		printf("I am in auto turn (%f %f) ", mathAngle, _angle);
#endif
		if(turnLeft == false){
#if DRIVE_AUTO_DEBUG
			printf("turn right\n");
#endif
			//powerTurn = .7 * ((mathAngle - _angle)/-(mathAngle - _angle));
			//Move_Speed(.5, -.5); // turn right
			_leftPowers = .8;
			_rightPowers = .5; //-.6
		}
		else if(turnLeft == true){
#if DRIVE_AUTO_DEBUG
			printf("turn left\n");
#endif
			//Move_Speed(-.5, .5); // turn left
			_leftPowers = .5;
			_rightPowers = .8;
		}
		if ((_angle > (mathAngle + DRIVE_AUTO_ANGLE_TOLERANCE) && turnLeft) ||  (_angle < (mathAngle - DRIVE_AUTO_ANGLE_TOLERANCE) && !turnLeft)){
#if DRIVE_AUTO_DEBUG
			printf("stop turning\n");
#endif
			//Move_Speed(0, 0);
			_leftPowers = 0;
			_rightPowers = 0;
			drive = kAuto;
		}

		//if (gyro.GetAngle() < initGyro + _angle){
			//_leftPowers = _maxSpeed;
		//}
		//if (gyro.GetAngle() > initGyro + _angle){
			//_rightPowers = _maxSpeed;
		//}
		//if ((gyro.GetAngle() >= initGyro && _angle < 0 )|| (gyro.GetAngle() <= initGyro && _angle > 0 )){
			//_rightPowers = 0;
			//_leftPowers = 0;
			//drive = kAuto;
		//}
		break;

		case kDisabled:
		break;
	case kAuto:
		default:
		break;
	}
	leftMotors.Set(_leftPowers * DRIVE_AUTO_LEFT_SCALE);
	rightMotors.Set(_rightPowers * DRIVE_AUTO_RIGHT_SCALE);
	_prevRight = _rightPowers;
	_prevLeft = _leftPowers;
}
void Drive::CalibrateGyro(){
	if(!_gyroCalibrated){
		for (int i=0; i < 3; i++){
			calibrate.Set(1);
			SmartDashboard::PutNumber("thunderdashboard_gyro", 1);
			Wait(.25);
			calibrate.Set(0);
			SmartDashboard::PutNumber("thunderdashboard_gyro", 0);
			Wait(.25);
			calibrate.Set(1);
			SmartDashboard::PutNumber("thunderdashboard_gyro", 1);
			Wait(.25);
			calibrate.Set(0);
			SmartDashboard::PutNumber("thunderdashboard_gyro", 0);
			Wait(.25);
		}
		calibrate.Set(1);
		SmartDashboard::PutNumber("thunderdashboard_gyro", 1);
		gyro.Calibrate();
		calibrate.Set(0);
		SmartDashboard::PutNumber("thunderdashboard_gyro", 0);
		_gyroCalibrated = true;
	}
}
