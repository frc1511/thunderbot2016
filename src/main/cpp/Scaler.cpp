#include "IOMap.h"
#include "Scaler.h"
#include <ctre/Phoenix.h>
//#define Normalized
#define SlowEnd
#define ScaleBroke
/*#ifdef robot_2
// Potentiometer reading for lowest rotation of pivot (volts)
const double kPivPotMin = 1.96;

// Potentiometer reading for max rotation of pivot (volts)
const double kPivPotMax = 4.9;

// Servo value for the locked position
const float kWinchServoLocked = .45;

// Servo value for the unlocked position
const float kWinchServoUnlocked = .08;

const float kNonNormalizedExt = -65;

// Number of encoder ticks when at top of full desired extension travel
const double kExtEncoderMax = 660;

//Position to keep hook away from bar
const float kPivFirstStop = .9;
*/

//#else
// Potentiometer reading for lowest rotation of pivot (volts)
const double kPivPotMin = .41;

// Potentiometer reading for max rotation of pivot (volts)
const double kPivPotMax = 3.87;

// Servo value for the locked position
const float kWinchServoLocked = .45;

// Servo value for the unlocked position
const float kWinchServoUnlocked = .08;
#ifdef ScaleBroke
const float kNonNormalizedExt = -25;
#else
const float kNonNormalizedExt = -35;
#endif
// Number of encoder ticks when at top of full desired extension travel
const double kExtEncoderMax = 645;

//Position to keep hook away from bar
const float kPivFirstStop = .9;


//#endif

// Number of encoder ticks above the zero point
// for the extension to stop going downward at
const double kExtEncoderMin = 0;

// Point of normalized full travel above which we use
// kExtFastMultiplier against desired speed values
// and below which we use kExtSlowMultiplier
const float kExtLockedInFastRange = .1;

// Point of normalized full travel above which we use
// kExtFastMultiplier against desired speed values
// and below which we use kExtSlowMultiplier
const float kExtInFastRange = .3;

// Same as kExtInFastRange, but reversed and for out instead of in
const float kExtOutFastRange = .91;

// Maximum amount of current scaler extension  can draw - forward and reverse
// In amps
const float kExtStallLimit = 90.0f;  //may need to be changed

// Normalized angle (0->1) at which pivot goes to neutral
const double kPivNeutralAngle = (2);

// Motor output at which pivot is driven (used for up and down)
const float kPivSpeedUp= 0.45f;

// Value multiplied against requested extension speed when
// extension is NOT nearing the end stops
const float kExtFastMultiplier = 1.0;

// Value multiplied against requested extension speed when
// extension is nearing the end stops
const float kExtSlowMultiplier = .3; // CHANGE BACK TO 2.5 AFTER RUCKUS

// Absolute maximum motor output limit of extension - forward and reverse
const float kExtSpeedMax = .35;

// Absolute maximum motor output limit of extension - forward and reverse
const float kExtBrokenSpeedMax = .18;

//see name
const float kExtSpeedMaxLift = 1;
const float kExtSpeedMaxLiftInPits = .2;


const float kPivSpeedDown = 0.1f;

const double kStallTime = 3.0;

const float kExtLockSlow = .8;

const float kExtLockCatch = .003;

const float kUpStopTime = 5.0;

const float kUpSlowTime = 1.5;

const float kUpSlowMultiply = .25;

const float kPivSlowMultiply = .35;

const float kBreacherDownEnough = .3;

const float kBreacherGoTo = .1;		//must be less than the one above

const float kBreacherGoToUp = .94;

/*
 *
 * FIXME LIST



pivot - :)brake mode when below vertical, coast when past vertical
        add stall current protection
  :) change pivot state to enum up/down/stop (var _pivotDirection)
  :) derive speed in process
  :) ******* Breacher pivot protection check doesn't have corresponding stop() -  treating as broken!
  :) do variable-based derivation, then set in one spot! start @ 0, change one met conditions only


extension -
  :) change mode/range derivation to be in process() only! Right now only changes on call to setfunction()
  :) make pivot_get_angle() check in which able to go up/down a constant!
  :) break out checks on limits, servo lock, pivot angle -- make sure "else" cases are handled properly!
   On stall - lock out that direction until other direction or stop COMMAND is seen


winch lock - on sending/setting lock, force winch to go stop/correct direction only!
Winch unlock -- add enforced unlocking sequence (don't let locked-direction action go until unlock-direction action goes for X seconds)

:) Move encoder reset FIRST in process!
 *
 */



Scaler::Scaler(Intake* intake):
pivot(CAN_ID_SCALER_PIVOT),
winch1(CAN_ID_SCALER_WINCH_A),
winch2(CAN_ID_SCALER_WINCH_B),
extende(DIG_IO_SCALER_WINCH_ENCODER_A, DIG_IO_SCALER_WINCH_ENCODER_B),
pot(ANALOG_IN_SCALER_PIVOT_POT),
lock(PWM_OUT_SCALER_WINCH_LOCK),
_pivotDirection(SCALE_STOP),
_armSpeed(0),
_armLock(0),
_pivUp(false),
_pivDown(false),
_extendBroke(false),
_pivBroke(false),
_extUp(false),
_extDown(false),
intake(intake),
neutral(false),
extStall(NOTSTALL),
stallCycleCount(0),
timer(),
_currentValues(),
_stallIdx(0),
_allUp(false),
UpStop(),
_hasBeenUp(false),
_waitForBreacher(false)
{
	_inPitMode = false;

	winch2.Set(ControlMode::Follower, CAN_ID_SCALER_WINCH_A);
	_prevAtLowerLimit = GetExtensionLowerAtLimit();
	pivot.SetNeutralMode(NeutralMode::Brake);
	pivot.SetInverted(true);
	timer.Start();
	Stall_Zero();
	UpStop.Start();
}

#define SCALER_PIVOT_DEBUG 0

void Scaler:: Process() {
	//printf("current %f \n " , winch1.GetOutputCurrent());
//------------------------------------------ENCODER---------------------------------------
	bool atLower = GetExtensionLowerAtLimit();
#ifdef ScaleBroke
	if (!_armLock){
		bool atLower = GetExtensionLowerAtLimit();
		if (!atLower && _prevAtLowerLimit){
			extende.Reset();
		}
	}
#else
	if (!atLower && _prevAtLowerLimit){
		extende.Reset();
	}
#endif
//------------------------------------------PIVOT-----------------------------------------
	Extension_At_Upper_End();	//calls this function so we know for the pivot range if we have already been up or not...
	Extension_At_Lower_End();	//...they both set a variable called _hasBeenUp which only changes if we are at the ends

	float wantPivSpeed;

	_prevAtLowerLimit = atLower;

	switch (_pivotDirection){
		case SCALE_STOP:
			wantPivSpeed = 0;
			break;
		case SCALE_UP:
			wantPivSpeed = kPivSpeedUp;
			break;
		case SCALE_DOWN:
			wantPivSpeed = -kPivSpeedDown;
			break;
	}
	float _pivotSet = 0;
	if(_pivBroke == false){						//if the pivot pot isn't broken...
#if SCALER_PIVOT_DEBUG
		printf("(%f,%f,%d) ", wantPivSpeed, Pivot_Get_Angle(), _hasBeenUp);
#endif
		if (Pivot_At_Upper_End() == false && Pivot_At_Lower_End() == false){	//and if the arm isn't at the ends of the range
#if SCALER_PIVOT_DEBUG
			printf("in between the end stops ");
#endif
			if(wantPivSpeed > 0){
#if SCALER_PIVOT_DEBUG
				printf("keep moving UP  ");
#endif
				if(_pivBroke){		//and the extension is either down and below the first stop or up)))
					_pivotSet = (wantPivSpeed);			//then set the pivot to up, down, or stop
				}
				else if(Pivot_Get_Angle() < kPivFirstStop){
					// breacher is out of our way, so we can move
					_pivotSet = (wantPivSpeed);			//then set the pivot to up, down, or stop
#if SCALER_PIVOT_DEBUG
					printf("before first stop ( middle)  ");
#endif
					if(!_hasBeenUp && Pivot_Get_Angle() > (kPivFirstStop - .15)){
						_pivotSet = _pivotSet * kPivSlowMultiply;
#if SCALER_PIVOT_DEBUG
						printf("slow down before stop ");
#endif
					}
				}
				else if(_hasBeenUp){
					_pivotSet = (wantPivSpeed);			//then set the pivot to up, down, or stop
#if SCALER_PIVOT_DEBUG
					printf("leaving first stop ");
#endif
				}
				else{
					_pivotSet = 0;
#if SCALER_PIVOT_DEBUG
					printf("at the first stop and scaler not all up ");
#endif
				}
			}
			else if(wantPivSpeed < 0){
#if SCALER_PIVOT_DEBUG
				printf("keep moving DONN  ");
#endif
				_pivotSet = wantPivSpeed;
			}
			else{
#if SCALER_PIVOT_DEBUG
				printf("stop  ");
#endif
				_pivotSet = 0;
			}
		}
		else if(Pivot_At_Upper_End() == true && wantPivSpeed <= 0){		//if the arm is all the way up...
#if SCALER_PIVOT_DEBUG
			printf("at upper end  ");
#endif
			_pivotSet = (wantPivSpeed);			//only let it move down or stop
		}
		else if(Pivot_At_Lower_End() == true && wantPivSpeed >= 0){		//same as above, but for down
#if SCALER_PIVOT_DEBUG
			printf("at lower end  ");
#endif
			_pivotSet = (wantPivSpeed);
		}
		else if(Pivot_At_Upper_End() && wantPivSpeed > 0 && Pivot_Get_Angle() < 1.5){
#if SCALER_PIVOT_DEBUG
			printf("move slow???  ");
#endif
			_pivotSet = .15;
		}
		else{
#if SCALER_PIVOT_DEBUG
			printf("stop moving  ");
#endif
			_pivotSet = 0;
		}
	}
	else if (_pivBroke == true){				//if the pivot pot is broken
		_pivotSet = (wantPivSpeed);		//just ignore it being all up or all down but still move
	}
	else{
		_pivotSet = 0;
	}
#if SCALER_PIVOT_DEBUG
	printf("\n");
#endif
	if(_pivBroke == false && Pivot_Get_Angle() < .55 && _pivotDirection == SCALE_DOWN){	//safety check
		//printf("saftey stop!!!\n");
		_pivotSet = 0;
	}

	// before we move up, let's make sure the breacher is out of our way
	if (!intake->Pivot_Broken() && (intake->Pivot_Get_Angle() >= kBreacherDownEnough) && ((Pivot_Get_Angle() < kPivFirstStop) && !_pivBroke ) && !(_pivotSet == 0)){
		_pivotSet = 0;		//stops it from moving if breacher up

		// only move it, if we are not already moving
		if (!_waitForBreacher) {
			_waitForBreacher = true;
			intake->Pivot_Go_To(kBreacherGoTo);		//puts the breacher down if the scaler tries to pivot up
#if SCALER_PIVOT_DEBUG
			printf("breacher, please go down %f\n", wantPivSpeed);
#endif
		}
	}

	//Stop pivot go to?
	if(_waitForBreacher) {
		if (intake->At_Pivot_Go_To()){
#if SCALER_PIVOT_DEBUG
			printf("breacher is now down\n");
#endif
			intake->Stop_Goto();
			_waitForBreacher = false;
		}
	}



	//printf("%f\n", (double)_pivotSet);
	/*
	if(wantPivSpeed > 0 && Pivot_Get_Angle() < .7 && !_pivBroke){
		_pivotSet = _pivotSet * 1.4;
	}
*/
pivot.Set(ControlMode::PercentOutput, _pivotSet);

//------------------------------------------------EXTENSION------------------------------------------------------

	float _winchSpeed = _armSpeed;		//takes the speed of the joystick n the all the other stuff changes it
	if (_extendBroke == false) {		//if the extend encoder isn't broken
		if(_armSpeed > 0 && Extension_Get() < kExtOutFastRange){	//checks to see what direction and where it's going
			_winchSpeed = (_armSpeed * kExtFastMultiplier);		//set both motors to a certain speed
		}
		if (_armSpeed < 0 && Extension_Get() > kExtInFastRange){
			_winchSpeed = (_armSpeed * kExtFastMultiplier);
		}
		if (_armSpeed > 0 && Extension_Get() >= kExtOutFastRange){
			_winchSpeed = (_armSpeed * kExtSlowMultiplier);
		}
		if (_armSpeed < 0 && Extension_Get() <= kExtInFastRange && !_armLock){
			_winchSpeed = (_armSpeed * kExtSlowMultiplier);
		}
		if (_armSpeed < 0 && Extension_Get() <= kExtLockedInFastRange && _armLock){
			_winchSpeed = (_armSpeed * kExtLockSlow);
		}
		if (_armSpeed > 0 && Extension_Get() >= 1.0){
			_winchSpeed = 0;
		}
#ifdef Normalized
		if ((Extension_At_Upper_End() && _armSpeed > 0)||(Extension_At_Lower_End() && _armSpeed < 0)){	//bunch of safety checks
#else
		if ((Extension_At_Upper_End() && _armSpeed > 0)||(Extension_At_Lower_End() && _armSpeed < 0 && !_armLock)){	//bunch of safety checks
#endif
			_winchSpeed = 0;																			//stops it if unsafe
				}
		else{			//if everything isn't broken or not gonna work...
				//just ignore everything and do speed from the joystick
		}
	}
	else{	//lock, pivot
		_winchSpeed = (_armSpeed);
	}
	if((!_pivBroke && Pivot_Get_Angle() < .5 &&_armSpeed > 0 ) || (_armLock && _armSpeed > 0)){	//safety, if the pivot isn't up or the arm is locked...
		_winchSpeed = 0;																		//don't extend the arm
	}
	if (_armLock == true){		//if the variable that indicates the arm should be locked is true...
		lock.Set(kWinchServoLocked);			//lock the arm and stop telling it to lock
	}
	else {
		lock.Set(kWinchServoUnlocked);
	}
	//printf ("current %f \n", winch1.GetOutputCurrent());
#ifdef Normalized
	if(_armLock && _winchSpeed < 0 && Extension_Get() < kExtLockCatch  && !_allUp){
		_winchSpeed = 0;
		//printf("%f Extension \n", (double)extende.Get());
		_allUp = true;
		UpStop.Reset();
		UpStop.Start();
#else
	if(_armLock && _winchSpeed < 0 && extende.Get() < kNonNormalizedExt && !_extendBroke){
		_winchSpeed = 0;
		if(!_allUp){
			//printf("%f Extension \n", (double)extende.Get());
			_allUp = true;
			UpStop.Reset();
			UpStop.Start();
		}
#endif
	}
	if(_allUp && UpStop.Get() < kUpStopTime){
		if(UpStop.Get() < kUpSlowTime){
			_winchSpeed = kUpSlowMultiply * _winchSpeed;
		}
		else{
			_winchSpeed = 0;
		}
	}
	else{
		_allUp = false;
	}
//---------------------------------------EXT STALL--------------------------------------------------------------------

	if(extStall != STALLUP && extStall != STALLDOWN){
		float avg = 0;
		_currentValues[_stallIdx] = winch1.GetOutputCurrent();		//sets a value in the array to the current output
		if(_stallIdx < STALLIDXNUM){				//if the number in the array isn't the last one...
			_stallIdx++;					//make it the next number
		}
		else{								//if it is the last one...
			_stallIdx = 0;					//start over
		}

		for(int i=0; i<STALLIDXNUM; i++){			//adds up all the numbers in the array
			avg =+ _currentValues[i];
		}
		avg = avg/STALLIDXNUM;					//averages all the numbers


		if(avg > kExtStallLimit){			//if the average is more than the stall limit...
			if(_winchSpeed > 0){			//and if its going up...
					extStall = STALLUP;		//set it to stall up
					timer.Reset();			//and reset the timer
					timer.Start();
			}
			else if(_winchSpeed <= 0){		//if it's going down...
					extStall = STALLDOWN;	//set it to stall down
					timer.Reset();
					timer.Start();//and reset the timer
			}
		}
	}
	switch(extStall){		//Checks if the motor is stalling and only lets it go after a certain amount of time
	case(STALLUP):
			printf("Has StalledUP");
			if(timer.Get() < kStallTime){
				_winchSpeed = 0;
			}
			else{
				Stall_Zero();
				extStall = NOTSTALL;
			}
		break;
	case(STALLDOWN):
			printf("Has StalledDown");
			if(timer.Get() < kStallTime){
				_winchSpeed = 0;
			}
			else{
				Stall_Zero();
				extStall = NOTSTALL;
			}
		break;
	case(NOTSTALL):
		break;
	}
		//Limits on the max speed of the extension
	if(_armLock && _winchSpeed < 0){
		if(_inPitMode){
			if(_winchSpeed < -kExtSpeedMaxLiftInPits){
				_winchSpeed = -kExtSpeedMaxLiftInPits;
			}
		}
		else {
			if(_winchSpeed < -kExtSpeedMaxLift){
				_winchSpeed = -kExtSpeedMaxLift;
			}
		}
	}
	else if (!_extendBroke) {
		if(_winchSpeed > kExtSpeedMax){
			_winchSpeed = kExtSpeedMax;
		}

		else if(_winchSpeed < -kExtSpeedMax){
			_winchSpeed = -kExtSpeedMax;
		}
	} else {
		if(_winchSpeed > kExtBrokenSpeedMax){
			_winchSpeed = kExtBrokenSpeedMax;
		}
		else if(_winchSpeed < -kExtBrokenSpeedMax){
			_winchSpeed = -kExtBrokenSpeedMax;
		}

	}
	winch1.Set(ControlMode::PercentOutput, _winchSpeed);

//----------------------------------------------------BRAKE/NEUTRAL--------------------------------------------------

		//Coasting and braking stuffs for when we are scaling
	if (Pivot_Get_Angle() >=  kPivNeutralAngle && !neutral ){	//&& _armSpeed > 0??? && Extension_Get() < .5???
		pivot.SetNeutralMode(NeutralMode::Coast);
		neutral = true;
	}
	else if(neutral && Pivot_Get_Angle() < kPivNeutralAngle){
		pivot.SetNeutralMode(NeutralMode::Brake);
		neutral = false;
	}
}
void Scaler:: Stall_Zero(){
	for(int i=0; i<STALLIDXNUM; i++){
		_currentValues[i]=0;
	}
	_stallIdx = 0;
}

void Scaler::Pivot_Arm(Direction direction){
_pivotDirection = direction;
}


bool Scaler:: Pivot_At_Upper_End(){
	if(Pivot_Get_Angle() >= 1 && !_pivBroke){
		return true;
	}
	else{
		return false;
	}
}


bool Scaler:: Pivot_At_Lower_End(){
	if(Pivot_Get_Angle() <= 0 && !_pivBroke){
		return true;
	}
	else{
		return false;
	}
}


float Scaler:: Pivot_Get_Angle(){
	return (GetPivotPot() - kPivPotMin) / (kPivPotMax - kPivPotMin);
}


float Scaler:: GetPivotPot(){
	return pot.GetVoltage();
}


void Scaler:: Extend_Arm(float speed){
	_armSpeed = speed;
}


bool Scaler:: Extension_At_Upper_End(){
	if(_extendBroke){
		_hasBeenUp = true;
		return false;
	}
	else if (Extension_Get() >= .98 || GetExtensionUpperAtLimit()){
		if(!_hasBeenUp) {
			//puts the breacher up when the scaler just gets to the top
			intake->Pivot_Go_To(kBreacherGoToUp);
			_waitForBreacher = true;
#if SCALER_PIVOT_DEBUG
			printf("has been up\n");
#endif
		}
		_hasBeenUp = true;
		return true;
	}
	else{
		return false;
	}
}


bool Scaler:: Extension_At_Lower_End(){
	if(_extendBroke){
		return false;
	}
	else if (Extension_Get() <= 0 || GetExtensionLowerAtLimit()){
		_hasBeenUp = false;
		return true;
	}
	else{
		return false;
	}
}


float Scaler:: Extension_Get(){
	return (extende.Get() - kExtEncoderMin) / (kExtEncoderMax - kExtEncoderMin);
}

bool Scaler::GetExtensionLowerAtLimit()
{
	return !winch1.GetSensorCollection().IsRevLimitSwitchClosed();
}

bool Scaler::GetExtensionUpperAtLimit()
{
	return !winch1.GetSensorCollection().IsFwdLimitSwitchClosed();
}

bool Scaler:: Is_Arm_Locked(){
	return _armLock;
}


void Scaler:: Lock_Arm(bool On){
	_armLock = On;
}


void Scaler:: Extension_Set_Broken(bool Broken){
	_extendBroke = Broken;
}


void Scaler:: Pivot_Set_Broken(bool Broken){
	_pivBroke = Broken;
}


void Scaler:: Reset(){
	winch1.Set(ControlMode::PercentOutput, 0);
	lock.Set(kWinchServoUnlocked);
	_armLock = false;
	pivot.Set(ControlMode::PercentOutput, 0);
	_allUp = false;
	_hasBeenUp = false;
	_waitForBreacher = false;
}



/* Call when doing Pit things
 * True is run at speeds safe for the pits, False is run speeds needed for match play
 */
void Scaler::RunInPitMode(bool pitMode)
{
	_inPitMode = pitMode;
}


void Scaler::Debug(Feedback *feedback) {
	const char sysName[] = "Scaler";
#define T_OR_F(a) ((a) ? "true" : "false")

	// Phoenix API no longer has "limit ok" methods, and no clear way to get normally closed/open,
	// so turning off "OK" prints for now
	feedback->send_Debug_String(sysName, "Extension Limit Up", "OK: %s  Closed: %s", "??", T_OR_F(winch1.GetSensorCollection().IsFwdLimitSwitchClosed()));
	feedback->send_Debug_String(sysName, "Extension Limit Up Status", "%s", winch1.GetSensorCollection().IsFwdLimitSwitchClosed() ? "Sees inner tube, will run" : "Not seeing inner tub, at/past limit");
	feedback->send_Debug_String(sysName, "Extension Limit Down", "OK: %s  Closed: %s", "??", T_OR_F(winch1.GetSensorCollection().IsRevLimitSwitchClosed()));
	feedback->send_Debug_String(sysName, "Extension Limit Down Status", "%s", winch1.GetSensorCollection().IsRevLimitSwitchClosed() ? "Beam not broken, will run" : "Beam broken, will NOT run");
	feedback->send_Debug_Double(sysName, "Extension", (double)Extension_Get());
	feedback->send_Debug_Double(sysName, "Extension Raw", extende.Get());
	feedback->send_Debug_String(sysName, "Pivot", "%f Raw %f V", (double)Pivot_Get_Angle(), (double)pot.GetVoltage());
	feedback->send_Debug_String(sysName, "Pivot Broken", "%s", T_OR_F (_pivBroke));
	feedback->send_Debug_String(sysName, "Extension Broken", "%s", T_OR_F(_extendBroke));
	feedback->send_Debug_String(sysName, "Extension Stalled", "%d", extStall);
}



