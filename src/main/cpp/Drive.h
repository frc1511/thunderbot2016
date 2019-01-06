#ifndef DRIVE_H
#define DRIVE_H

#include <Spark.h>
#include <frc/Encoder.h>
#include <AnalogGyro.h>
#include "Feedback.h"

using namespace frc;

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

	void Auto_Straight(float maxSpeed, float distance); // 0 is stopped, 1 is full forward, do not pass negative. distance in inches - negative is backwards

	/* called during autonomous to tell the robot to go near a certain speed to a certain distance(100% straight)
	 * maxSpeed receives a number from 0 - 1, 0 being stopped, 1 being full speed forward
	 * distance receives a number in inches, negative being backward
	 * should use encoders to see distance, and gyro to keep it straight
	 */

	void Auto_Turn(float angle); //speed: 0 is stopped, 1 is full forward, do not pass a negative angle: negative numebers are left, positive right, and 0 does nothing

	/* called during autonomous to tell the robot to turn a certain number of degrees near a speed
	 * maxSpeed receives a number from 0 - 1, 0 being stopped, 1 being full speed forward
	 * angle receives any number, from -infinity to infinity, and will turn that number of degrees (numbers higher than 360 spin multiple times) -/+ correspond to left/right, respectively
	 */
	void Auto_Turn_Moat(float angle);
	bool Auto_Move_Complete(); //true if Auto_Turn() Auto_Drive() has completed, false if in progress

	/* tells the user if the last autonomous command was completed
	 * returns false if Auto_Straight()/Auto_Turn() ARE currently in process
	 * Returns true if neither of the autonomous functions are in process
	 */

	void Debug(Feedback *feedback);


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
	Spark leftMotors;
	Spark rightMotors;
	Encoder leftEncoder;
	Encoder rightEncoder;
	AnalogGyro gyro;
	DigitalOutput calibrate;
	float _leftPowers;
	float _rightPowers;
	float _maxSpeed;
	float _distance;
	float _angle;
	float _prevRight;
	float _prevLeft;
	double initGyro;
	bool _gyroCalibrated;
	float mathAngle;
	int powerTurn;
	bool turnLeft;

	typedef enum {kTeleop, kAuto, kAutoStraight, kAutoTurn, kAutoTurnMoat, kDisabled} process;
	process drive;
};

#endif

