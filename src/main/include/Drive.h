#pragma once

#include <frc/motorcontrol/Spark.h>
#include "IOMap.h"

#include <frc/Encoder.h>
#include <frc/AnalogGyro.h>

class Drive{

public:

	Drive();
	/*Constructor to begin at start of auto and teleop
	 */

	void Move_Speed(float leftSpeed, float rightSpeed); // 0 is stopped, 1 is full forward, -1 is full backward

	/* Is called whenever someone wants to change the speed of the robot
	 * tells the left and right motors what speeds to go at
	 * leftSpeed and rightSpeed receive a value of -1 to 1, negative numbers being backward, positive numbers being forward, and 0 being stopped
	 * leftSpeed is the speed of the left motors
	 * rightSpeed is the speed of the right motors
	 */

	void Reset();

	/* calls all functions and variables and sets them to their starting values
	 * Should be used at the beginning of Tele-op, autonomous, enable/disable, ETCETC
	 */


	void Process();// Debug goes in process

	/* Where all of the equations, motor setting, and general processing of data and inputs/outputs should go
	 * Other functions should just set an action / process to begin, and process will make the process happen
	 * all other functions are called once
	 * Process will be called continously
	 * contains debugging code
	 */

	void CalibrateGyro();
private:
	float RampPower(double previousPower, double wantedPower);
	void SetLeftMotors(float DrivePower);
	void SetRightMotors(float DrivePower);
	frc::Spark leftMotors {PWM_OUT_DRIVE_LEFT};
	frc::Spark rightMotors {PWM_OUT_DRIVE_RIGHT};
	frc::Encoder leftEncoder {DIG_IO_DRIVE_SHAFT_ENCODER_LEFT_A, DIG_IO_DRIVE_SHAFT_ENCODER_LEFT_B};
	frc::Encoder rightEncoder {DIG_IO_DRIVE_SHAFT_ENCODER_RIGHT_A, DIG_IO_DRIVE_SHAFT_ENCODER_RIGHT_B};
	// frc::DigitalOutput calibrate;
	float _leftPowers = 0;
	float _rightPowers = 0;
	// float _maxSpeed;
	// float _distance;
	// float _angle;
	float _prevRight = 0;
	float _prevLeft = 0;
	// float mathAngle;
	// int powerTurn;
	// bool turnLeft;
};