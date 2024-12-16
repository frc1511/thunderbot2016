#include "Intake.h"
#include "IOMap.h"

const double BEATER_BAR_SPEED_IN = .65;     	//Speed of Intake Going In
const double BEATER_BAR_SPEED_OUT = -.8;   	//Speed of Intake Going Out
const double PIVOT_LOWER_LIMIT_ANGLE = 0;  	//normalized value of the low value of the pivot potentiometer, dont mess with this
const double PIVOT_UPPER_LIMIT_ANGLE = .98;	//normalized value of the High value of the pivot potentiometer, dont mess with this
const double PIVOT_GOTO_DRAWBRIDGE = .32;  	//value we need to go to for the Drawbridge, set when controls uses Drawbridge_Value()
//#ifdef robot_2
//const double PIVOT_POT_HIGH_VALUE = 2.98;   	//absolute value of the high point of the breacher, change this when we change the breacher
//const double PIVOT_POT_LOW_VALUE = 0.34;    	//absolute value of the low point of the breacher, change this when we change the breacher
//#else
const double PIVOT_POT_HIGH_VALUE = 4.14;//3.13 4.33;   	//absolute value of the high point of the breacher, change this when we change the breacher
const double PIVOT_POT_LOW_VALUE = 1.4;//.32 1.78;    	//absolute value of the low point of the breacher, change this when we change the breacher
//#endif

// TORONTO #'s
#if 0
const float MAX_PIVOT_SPEED_UP = 0.3f;          //max speed of the pivot going up
const float MAX_PIVOT_SPEED_DOWN = -.3f;          //max speed of the pivot going down
const float MAX_PIVOT_GRAV_SPEED_UP = 0.45f; 	//speed for when we are fighting gravity
const float MAX_PIVOT_GRAV_SPEED_DOWN = -0.4f;
#endif

// New #'s for breacher with springs!
const float MAX_PIVOT_SPEED_UP = 0.325f;          //max speed of the pivot going up  .225
const float MAX_PIVOT_SPEED_DOWN = -.4;          //max speed of the pivot going down   -.4
const float MAX_PIVOT_GRAV_SPEED_UP = 0.7f; 	//speed for when we are fighting gravity    .6
const float MAX_PIVOT_GRAV_SPEED_DOWN = -0.75f;					// -.75
const float MAX_PIVOT_SPEED_UP_SLOW = .3;						//.2
const float MAX_PIVOT_SPEED_DOWN_SLOW = -.1;					//-.2

#define INTAKE_PIVOT_DEBUG 0


Intake::Intake():// construction, in same order as .h
		// PivotMotor, NOTE: Moved these into .h for now
		// BeaterBarMotor,
		// BeamBreak,
		// UpperLimit,
		// LowerLimit,
		// PivotPot,
	    // _desiredPivotSpeed,
		// _beaterBarSpeed,
		// _desiredGotoAngle,
		// _pivotBroken,
		// _beaterBroken,
		// isAtPosition,
		_pivotControlMode(MANUAL)
		{
	BeaterBarMotor.SetInverted(true);
	PivotMotor.SetInverted(true);
	// BeaterBarMotor.ConfigOpenloopRamp(0.3, 0); // NOTE: I have left this in for now, as the below translation into cansparkmax may not be accurate. Remove later when solution found
	BeaterBarMotor.SetOpenLoopRampRate(.3);
	SetNeutral(true);
	_desiredPivotSpeed = 0;
	_beaterBarSpeed = 0;
	_pivotControlMode = MANUAL;
	isAtPosition = false;
}

void Intake::SetNeutral(bool neutral){
	ctre::phoenix::motorcontrol::NeutralMode ctrenmode = neutral ? ctre::phoenix::motorcontrol::NeutralMode::Coast : ctre::phoenix::motorcontrol::NeutralMode::Brake;
	rev::CANSparkMax::IdleMode revnmode = neutral ? rev::CANSparkMax::IdleMode::kCoast : rev::CANSparkMax::IdleMode::kBrake;
	BeaterBarMotor.SetIdleMode(revnmode); // NOTE: Needs a burn flash?
	PivotMotor.SetNeutralMode(ctrenmode);
}

void Intake::Reset(){
	_desiredPivotSpeed = 0;
	_beaterBarSpeed = 0;
	BeaterBarMotor.Set(0);
	PivotMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
	_pivotControlMode = MANUAL;
	isAtPosition = false;
}

void Intake::Process() {
	/* Take Values set by other functions and calculate the final value */
	switch (_pivotControlMode) {
		case MANUAL:										//If the last command was a manual input
			if ( _desiredPivotSpeed > 0 && Pivot_At_Upper_Stop()){
				_desiredPivotSpeed = 0;								//If the Breacher pivot is hitting the upper hard stop and we are going up
#if	INTAKE_PIVOT_DEBUG
				printf("upper stop manual\n");
#endif
			}
			if ( _desiredPivotSpeed < 0 && Pivot_At_Lower_Stop()){
				_desiredPivotSpeed = 0;								//If the Breacher pivot is hitting the lower hard stop and we are going down
#if	INTAKE_PIVOT_DEBUG
				printf("lower stop manual\n");
#endif
			}
			break;
		case GOTO:													//if the last command was a goto command
			//if (!isAtPosition) {
				if (Pivot_Get_Angle() > _desiredGotoAngle + .1){
					_desiredPivotSpeed = -.5f;
#if	INTAKE_PIVOT_DEBUG
					printf("goto down\n");
#endif
				}else{
					if (Pivot_Get_Angle() < _desiredGotoAngle - .1){
							_desiredPivotSpeed = .6f;
#if	INTAKE_PIVOT_DEBUG
							printf("goto up\n");
#endif
					}else{
						_desiredPivotSpeed = 0;
						isAtPosition = true;
#if	INTAKE_PIVOT_DEBUG
						printf("at position\n");
#endif
					}
				}
			//} else {
			//	_desiredPivotSpeed = 0;
			//}
			if ( _desiredPivotSpeed > 0 && Pivot_At_Upper_Stop()){
				_desiredPivotSpeed = 0;		//If the Breacher pivot is hitting the upper hard stop and we are going up
#if	INTAKE_PIVOT_DEBUG
				printf("upper stop\n");
#endif
			}
			if ( _desiredPivotSpeed < 0 && Pivot_At_Lower_Stop()){
				_desiredPivotSpeed = 0;
#if	INTAKE_PIVOT_DEBUG
				printf("lower stop\n");
#endif
			}
			break;
		default:
			break;
		}

	// Absolute max speeds
	if (!_pivotBroken && (Pivot_Get_Angle() < .65)){
		if (_desiredPivotSpeed > MAX_PIVOT_GRAV_SPEED_UP){
			_desiredPivotSpeed = MAX_PIVOT_GRAV_SPEED_UP;
		}
	} else if (_desiredPivotSpeed > MAX_PIVOT_SPEED_UP)
			_desiredPivotSpeed = MAX_PIVOT_SPEED_UP;

	if (!_pivotBroken) {
		if (Pivot_Get_Angle() > .55) {
			if (_desiredPivotSpeed < MAX_PIVOT_GRAV_SPEED_DOWN)
				_desiredPivotSpeed = MAX_PIVOT_GRAV_SPEED_DOWN;
		} else if (Pivot_Get_Angle() > .2) {
			if (_desiredPivotSpeed < MAX_PIVOT_SPEED_DOWN)
				_desiredPivotSpeed = MAX_PIVOT_SPEED_DOWN;
		} else {
			if (_desiredPivotSpeed < MAX_PIVOT_SPEED_DOWN_SLOW)
				_desiredPivotSpeed = MAX_PIVOT_SPEED_DOWN_SLOW;
		}
	} else {
		if (_desiredPivotSpeed < MAX_PIVOT_SPEED_DOWN)
			_desiredPivotSpeed = MAX_PIVOT_SPEED_DOWN;
	}


	//printf("%f Beaterafter \n", (double)_beaterBarSpeed);
	if (_beaterBarSpeed > 0 && /*!BeamBreak.Get()) && */!_beaterBroken){
		_beaterBarSpeed = 0;										//if we broken or we got a ball and we tryin an roll balls in
	}
	//printf("%f BeaterB4", (double)_beaterBarSpeed);
	/*
	Apply Values
	*/
	BeaterBarMotor.Set(_beaterBarSpeed); //Applies calculated values of _beaterBarSpeed to the Beater Bar Motor
	PivotMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, _desiredPivotSpeed); //Applies calculated value of _desiredPivotSpeed to the Pivot Motor
	/*DEBUG*/

}

void Intake::Pivot(float speed){// as it says in Pivot.h, just sets a variable with the speed, which is dealt with in process
	_desiredPivotSpeed = speed;
	if (speed != 0) {
		_pivotControlMode = MANUAL;
#if	INTAKE_PIVOT_DEBUG
		printf("breacher goto manual %f\n", speed);
#endif
	}
}

bool Intake::Pivot_At_Upper_Stop(){//as it says in Intake.h
	return UpperLimit.Get() || (Pivot_Get_Angle() >= PIVOT_UPPER_LIMIT_ANGLE);
}

bool Intake::Pivot_At_Lower_Stop(){//as it says in Intake.h
	return LowerLimit.Get() || (Pivot_Get_Angle() <= PIVOT_LOWER_LIMIT_ANGLE);
}

float Intake::Pivot_Get_Angle(){//as it says in Intake.h
	return (PivotPot.Get() - PIVOT_POT_LOW_VALUE) / (PIVOT_POT_HIGH_VALUE - PIVOT_POT_LOW_VALUE);
}

bool Intake::Is_Ball_Acquired(){//as it says in Intake.h
	return true;//!BeamBreak.Get();
}

void Intake::Beater_Bar(BeaterBarDirection direction){
	switch (direction) {//takes the direction variable and sets the beater bar to a constant based on it
	case IN:
		_beaterBarSpeed = BEATER_BAR_SPEED_IN;
		break;
	case OUT:
		_beaterBarSpeed = BEATER_BAR_SPEED_OUT;
		break;
	case STOP:
		_beaterBarSpeed = 0;
		break;

	}
}

void Intake::Pivot_Set_Broken(bool broken){//sets a variable based on what the person inputs
	_pivotBroken = broken;
	if (broken && _pivotControlMode == GOTO) {
		isAtPosition = true;
		_desiredPivotSpeed = 0;
	}
}
bool Intake::Pivot_Broken(){
	return _pivotBroken;
}
void Intake::Beater_Bar_Set_Broken(bool broken){//sets a variable based on what the user inputs
	_beaterBroken = broken;
}

void Intake::Pivot_Go_To(float angle){//sets the angle we would like to drive the pivot to, and sets the process mode to goto
	_pivotControlMode = GOTO;
	_desiredGotoAngle = angle;
	if (!_pivotBroken){
		isAtPosition = false;
	} else {
		isAtPosition = true;
	}
#if	INTAKE_PIVOT_DEBUG
	printf("intake go to: %f %d %d\n", angle, isAtPosition, _pivotBroken);
#endif
}

bool Intake::At_Pivot_Go_To(){//returns false if we are in  the goto mode, and true if we are not
	switch (_pivotControlMode){
	case (MANUAL):
			return true;
	case (GOTO):
			return isAtPosition;
	default:
			return true;
	}
}

void Intake::Drawbridge_Position(){
	Pivot_Go_To(PIVOT_GOTO_DRAWBRIDGE);
}

void Intake::Stop_Goto(){
	_pivotControlMode = MANUAL;
#if	INTAKE_PIVOT_DEBUG
	printf("intake (aka breacher) must stop now\n");
#endif
}