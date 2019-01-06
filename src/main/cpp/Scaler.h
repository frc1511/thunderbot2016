#ifndef SCALER_H_
#define SCALER_H_
#define STALLIDXNUM 20

#include "frc/WPILib.h"
#include <ctre/Phoenix.h>
#include "feedback.h"
#include "Intake.h"

using namespace frc;

class Scaler {


public:

	typedef enum { SCALE_STOP, SCALE_UP, SCALE_DOWN } Direction;
	typedef enum { STALLUP, STALLDOWN, NOTSTALL} Stall;

	void Pivot_Arm(Direction direction);

	/* Tells the Scaler to Pivot in a certain direction
	 * If direction is DOWN, the Scaler should pivot towards the down
	 * If direction is UP, the Scaler should pivot towards the up
	 * If direction is STOP, the Scaler should stop pivoting
	 * Should not go beyond the Upper and Lower limits
	 */

	bool Pivot_At_Upper_End();

	/* tells the user if the Scaler is at the upper end of the pivot motion
	 * Returns True if the Pivot is at the Upper limit
	 * Returns False if it is anywhere else
	 */

	bool Pivot_At_Lower_End();

	/* tells the user if the Scaler is at the upper end of the pivot motion
	 * Returns True if the pivot is at the lower limit
	 * Returns false if it is anywhere else
	 */

	float Pivot_Get_Angle(); // Number between 0(down) and 1(up)

	/* tells the user where the Scaler pivot is exactly, on a scale from 0 to 1
	 * If the pivot is at its lower limit, it will display 0
	 * If the pivot is at its upper limit, it will display 1
	 * mapped out to the rest of the positions, using the potentiometers value mapped out on a scale between 1 and 0
	 */

	void Extend_Arm(float speed); // + is Up, - is Down

	/* Tells the Scaler to Extend/Retract at a specified speed
	 * If the speed is negative, the Scaler will retract
	 * If the speed is positive, the Scaler will extend
	 * if the speed is zero, the Scaler will stop
	 */

	bool Extension_At_Upper_End();

	/* Tells the user if the Scaler has extended to its upper limit
	 * Returns True if the Scaler is fully extended
	 * Returns False if the Scaler is not fully extended
	 */

	bool Extension_At_Lower_End();

	/* Tells the user if the Scaler has retracted to its lower limit
	 * Returns True if the Scaler is in its most compressed state
	 * Returns False if the Scaler is at all extended
	 */

	float Extension_Get(); // number between 0(down) and 1(up)

	/* Tells the user the amount the Scaler has extended
	 * Returns a 0 if the Scaler is in its most compressed/retracted state
	 * Returns a 1 if the Scaler is in its most Extended state
	 */

	void Lock_Arm(bool On); // True is Lock, False is Unlock

	/* Tells the Scaler to lock/unlock its extension position
	 * If on is True, move the lock servo to a specified postion in order to lock its position,
	 * should anyone from telling the Scaler to extend
	 * If on is False, move the servo to a specified position in order to remove the lock
	 */

	bool Is_Arm_Locked();

	/* tells the user whether or not the arm has been locked
	 * Return True if the servo is currently in locking position
	 * Return False if the servo is currently in unlocked position
	 */

	void Extension_Set_Broken(bool broken); //true is broken, false is not broken

	/*	is called (with an input of True) when the pot sensor is broken
	 * 	should modify process to not use this sensor, and just use manual control
	 */

	void Pivot_Set_Broken(bool broken); //true is broken, false is not broken

	/*	is called (with an input of True) when the pot sensor is broken
	 * 	should modify process to not use this sensor, and just use manual control
	 */

	void Reset();

	/* calls all functions and variables and sets them to their starting values
	 * Should be used at the beginning of Tele-op, autonomous, enable/disable, ETCETC
	 */

	void Process();
	/* Where all of the equations, motor setting, and general processing of data and inputs/outputs should go
	 * Other functions should just set an action / process to begin, and process will make the process happen
	 * all other functions are called once
	 * Process will be called continuously
	 * contains debugging code
	 */


	/* Call when doing Pit things
	 */
	void RunInPitMode(bool pitMode); // True is run at speeds safe for the pits, False is run speeds needed for match play

	void Debug(Feedback *feedback);

	void Stall_Zero();

	Scaler(Intake* intake);
	//something Austin said to add when trying to figure out how to add motors... Idk what it does but oh well...
private:
	float GetPivotPot();

	bool GetExtensionLowerAtLimit();
	bool GetExtensionUpperAtLimit();


	TalonSRX pivot;
	TalonSRX winch1;
	TalonSRX winch2;
	Encoder extende;
	AnalogInput pot;
	Servo lock;

	Direction _pivotDirection;
	float _armSpeed;
	bool _armLock;
	bool _pivUp;
	bool _pivDown;
	bool _extendBroke;
	bool _pivBroke;
	bool _extUp;
	bool _extDown;

	bool _inPitMode;

	bool _prevAtLowerLimit;
	Intake* intake;
	bool neutral;
	Stall extStall;
	int stallCycleCount;
	Timer timer;
	float _currentValues[STALLIDXNUM];
	int _stallIdx;
	bool _allUp;
	Timer UpStop;
	bool _hasBeenUp;
	bool _waitForBreacher;

};
#endif
