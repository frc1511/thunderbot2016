#ifndef INTAKE_H_
#define INTAKE_H_

#include "frc/WPILib.h"
#include <ctre/Phoenix.h>
#include "Feedback.h"

using namespace frc;

class Intake {
	public:

	typedef enum {IN, OUT, STOP} BeaterBarDirection;

	Intake();

	/*debug*/
	void Debug(Feedback *feedback);

	/* tells the intake to rotate at a specified speed
	 *  Takes in speed you want to move the Pivot
	 * Speed takes a value of -1 to 1
	 * if negative, the pivot will spin toward the floor
	 * if positive, the pivot will spin toward the sky
	 * If the pivot is at upper stop, should not go up
	 * if the pivot is at lower stop, should not go down
	 */
	void Pivot(float speed);

	/* tells user if the intake has hit the lower limit
	 * Returns True if Intake is currently at the most vertical position
	 * Returns false if it is not at the most vertical position
	 */
	bool Pivot_At_Upper_Stop();

	/* tells user if the intake has hit the lower limit
	 * Returns True if Intake is currently at the most horizontal position
	 * Returns False if it is not at the most horizontal position
	 */
	bool Pivot_At_Lower_Stop();

	/* tells user where the intake is
	 * Returns a value between 0 and 1
	 * This value should span between the most horizontal and vertical position the Intake will be at
	 * If it is at o, the Intake will be at its most horizontal position
	 * if it is at 1, the Intake will be at its most vertical position
	 * should get a value from a potentiometer and span the value to be less than or equal to 1
	 */
	float Pivot_Get_Angle();

	/* Tells user if the Ball is breaking the beam break
	 * Returns True if the beam break in the hard stop is broken
	 */
	bool Is_Ball_Acquired();

	/* Tells the beater bar to move in a specific direction
	 * If direction is IN, the beater bar will take balls in at a set speed
	 * if direction is OUT, the beater bar will spit balls out at a set speed
	 * if direction is STOP, the beater bar will stop
	 * if the beam break is broken, should not take in balls
	 */
	void Beater_Bar(BeaterBarDirection direction);

	/*	is called (with an input of True) when the pot sensor is broken
	 * should modify process to not use this sensor, and just use manual control
	 */
	void Pivot_Set_Broken(bool broken); //True If broken, False if not

	/*	is called (with a input of True) when the beam break sensor is broken
	 * 	should modify process to not use this sensor, and just use manual control
	 */
	bool Pivot_Broken();
	//returns if the pivot is broken or not

	void Beater_Bar_Set_Broken(bool broken); //true if broken, false if not

	/* tells the Pivot to move to a specific angle, mapped out to a value from 1 to 0
	 * 1 should move it to the very top, 0 should move it to the very bottom ( this value is the same as Pivot_Get_Angle())
	 * After you are done with this function you MUST call Stop_Goto()
	 */
	void Pivot_Go_To(float angle); // Put in value from 0(horizontal) to 1(vertical)

	/* tells the user if we have completed a Pivot_Go_To() command
	 * Returns true if we are NOT currently doing a Pivot_Go_To command
	 * Returns false if we are currently doing a Pivot_Go_To command
	 */
	bool At_Pivot_Go_To(); //returns True if Pivot_Go_To() is done, false if not

	/* calls all functions and variables and sets them to their starting values
	 * Should be used at the beginning of Tele-op, autonomous, enable/disable, ETCETC
	 */
	void Reset(); // stops all movement.

	/* Where all of the equations, motor setting, and general processing of data and inputs/outputs should go
	 * Other functions should just set an action / process to begin, and process will make the process happen
	 * all other functions are called once
	 * Process will be called continously
	 */
	void Process();

	/*
	 * My function to call for drawbridge
	 */
	void Drawbridge_Position();

	/*
	 * For Controls when y button is unpressed, or for autonomous once we have completed a goto command, please do this before autonomous ends
	 */
	void Stop_Goto();
	private:
	ctre::phoenix::motorcontrol::can::TalonSRX PivotMotor;
	ctre::phoenix::motorcontrol::can::TalonSRX BeaterBarMotor;
	DigitalInput BeamBreak;
	DigitalInput UpperLimit;
	DigitalInput LowerLimit;
	AnalogInput PivotPot;

	float _desiredPivotSpeed;
	float _beaterBarSpeed;
	float _desiredGotoAngle;
	bool _pivotBroken;
	bool _beaterBroken;
	typedef enum {GOTO, MANUAL} pivotControl;

	bool isAtPosition;
	/*
	 * Set all motors to neutral for easy manual manipulation
	 */
	void SetNeutral(bool neutral);

	pivotControl _pivotControlMode;


};
#endif
