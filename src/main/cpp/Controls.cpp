#include "Controls.h"

#define JOYSTICK_DRIVER 0
#define JOYSTICK_AUX 1
#define JOYSTICK_BROKEN 2

#define BROKEN_BREACHER_PIVOT 1
#define BROKEN_BREACHER_INTAKE 2
#define BROKEN_ARM_ROTATE 3
#define BROKEN_ARM_TELESCOPE 4
#define BROKEN_IN_PIT_MODE 6
#define BROKEN_USE_TANK 7

// XBOX controller mapping
// axis
// 0 - Left stick left/right
// 1 - Left stick up/down
// 2 - Left trigger (+ only)
// 3 - Right trigger (+ only)
// 4 - Right stick left/right
// 5 - Right stick up/down
// buttons
// 1 - A
// 2 - B
// 3 - X
// 4 - Y
// 5 - left bumper
// 6 - right bumper
// 7 - dual box
// 8 - 3 bars
// 9 - left stick
// 10 - right stick
// POV - D-pad
#define DRIVER_LEFT_Y_AXIS 1			// up/down
#define DRIVER_RIGHT_X_AXIS 4      // left/right
#define DRIVER_RIGHT_Y_AXIS 5
#define DRIVER_LEFT_SLOW_BUTTON 5
#define DRIVER_RIGHT_SLOW_BUTTON 6
#define DRIVER_LEFT_TRIGGER_AXIS 2
#define DRIVER_RIGHT_TRIGGER_AXIS 3
#define DRIVER_SWAP_DRIVE_BUTTON 1
#define DRIVER_SWAP_CAMERAS_BUTTON 4


#define AUX_LEFT_Y_AXIS 1
#define AUX_RIGHT_Y_AXIS 5
#define AUX_INTAKE_IN_BUTTON 6
#define AUX_SCALER_UP_BUTTON 3
#define AUX_SCALER_DOWN_BUTTON 2
#define AUX_SCALER_UP_POV 270
#define AUX_SCALER_DOWN_POV 90
#define AUX_INTAKE_OUT_AXIS 3
#define AUX_ARM_LOCK_POV 180
#define AUX_ARM_UNLOCK_POV 0
#define AUX_ARM_LOCK_AXIS 2
#define AUX_ARM_UNLOCK_BUTTON 5
#define AUX_SWAP_CAMERAS_BUTTON 1
#define AUX_INTAKE_DRAWBRIDGE 4

// constructor
Controls::Controls(Drive *drive, Intake *intake)
{
	// save given object pointers, for later use
	_intake = intake;
	_drive = drive;

	// create broken switch objects
	_brokenJoystick = new Joystick(JOYSTICK_BROKEN);
	_brokenBreacherPivot = new ControlsButton(_brokenJoystick, BROKEN_BREACHER_PIVOT);
	_brokenBreacherIntake = new ControlsButton(_brokenJoystick, BROKEN_BREACHER_INTAKE);
	// _brokenArmRotate = new  ControlsButton(_brokenJoystick, BROKEN_ARM_ROTATE);
	// _brokenArmTelescope = new ControlsButton(_brokenJoystick, BROKEN_ARM_TELESCOPE);
	// _brokenInPitMode = new ControlsButton(_brokenJoystick, BROKEN_IN_PIT_MODE);

	// create driver controller objects
	_driverJoystick = new Joystick(JOYSTICK_DRIVER);
	_driverSwapDrive = new ControlsButton(_driverJoystick, DRIVER_SWAP_DRIVE_BUTTON);
	_swapDrive = 0;

	// create aux controller objects
	_auxJoystick = new Joystick(JOYSTICK_AUX);
// #if 0
// 	_auxScalerUp = new ControlsButton(_auxJoystick, AUX_SCALER_UP_BUTTON);
// 	_auxScalerDown = new ControlsButton(_auxJoystick, AUX_SCALER_DOWN_BUTTON);
// #else
// 	_auxScalerUp = new ControlsButton(_auxJoystick, AUX_SCALER_UP_POV,  true);
// 	_auxScalerDown = new ControlsButton(_auxJoystick, AUX_SCALER_DOWN_POV, true);
// #endif
	_auxIntakeDrawbridge = new ControlsButton(_auxJoystick, AUX_INTAKE_DRAWBRIDGE);


	// initialize dashboard items
	_dashboardInitialized = false;
	_breacherPosition = 0;
	_haveBall = 0;
	// _armLocked = 0;
	// _frontCamera = 0;
	_driveReverse = 0;
}

// deconstructor
Controls::~Controls()
{
}

// Called in Tele_Op and Disabled and Auto, Broken switch, Xbox controller, Updates dashboard (He is the man for thee job)
void Controls::Process(RobotMode robotmode)
{
	// process broken switches
	ProcessBroken();

	// process controllers
	if (robotmode == TELE_OP)
	{
		ProcessControllerDriver();
		ProcessControllerAux();
	}

	// update dashboard
	ProcessDashboard();
}

//
void Controls::ProcessControllerDriver()
{
	float driveLeftY;
	float driveRightX;
	bool slowLeft;
	bool slowRight;
	bool turboLeft;
	bool turboRight;
	float motorDriveLeft;
	float motorDriveRight;

	driveLeftY = -GetPosition(_driverJoystick, DRIVER_LEFT_Y_AXIS); // joystick Y values are inverted
	driveRightX = GetPosition(_driverJoystick, DRIVER_RIGHT_X_AXIS);
	turboLeft = (GetPosition(_driverJoystick, DRIVER_LEFT_TRIGGER_AXIS) > 0);
	turboRight = (GetPosition(_driverJoystick, DRIVER_RIGHT_TRIGGER_AXIS) > 0);
	slowLeft = _driverJoystick->GetRawButton(DRIVER_LEFT_SLOW_BUTTON);
	slowRight = _driverJoystick->GetRawButton(DRIVER_RIGHT_SLOW_BUTTON);

	// determine type of drive to do
	if (_brokenJoystick->GetRawButton(BROKEN_USE_TANK))
	{
		// use tank drive
		bool driveRightY = -GetPosition(_driverJoystick, DRIVER_RIGHT_Y_AXIS); // joystick Y values are inverted
		motorDriveLeft = GetPower(driveLeftY, slowLeft, turboLeft);
		motorDriveRight = GetPower(driveRightY, slowRight, turboRight);
	}
	else
	{
		// use default mode
		if (driveRightX == 0)
		{
			// drive forward and back
			motorDriveLeft = GetPower(driveLeftY, slowLeft, turboLeft);
			motorDriveRight = GetPower(driveLeftY, slowRight, turboRight);
		}
		else if (driveLeftY == 0)
		{
			// spins left and right
			motorDriveLeft = GetPowerTurn(driveRightX, slowLeft, turboLeft);
			motorDriveRight = -GetPowerTurn(driveRightX, slowRight, turboRight);
		}
		else
		{
			// arching - so set both to same speed (for now)
			motorDriveLeft = GetPower(driveLeftY, slowLeft, turboLeft);
			motorDriveRight = GetPower(driveLeftY, slowRight, turboRight);

			// decrease one side, scale it by how much the right stick is moved
			if (driveRightX > 0)
			{
				// arcing right (decrease right motor)
				motorDriveRight *= (1-driveRightX);
			}
			else
			{
				// arcing left (decrease left motor)
				// NOTE: invert joystick so we have a positive value for the scale factor
				motorDriveLeft *= -(-1-driveRightX);
			}
		}
	}

	// see if drive needs to be swapped
	if (_driverSwapDrive->Process() && _driverSwapDrive->Pressed())
	{
		if (_swapDrive == 0)
		{
			_swapDrive = 1;
		}
		else
		{
			_swapDrive = 0;
		}
	}

	// if drive needs to be swapped, then just negate the power
	if (_swapDrive == 1)
	{
		motorDriveLeft = -motorDriveLeft;
		motorDriveRight = -motorDriveRight;
	}

	_drive->Move_Speed(motorDriveLeft, motorDriveRight);
}

void Controls::ProcessControllerAux()
{
	float auxBreacherPivot;
	Intake::BeaterBarDirection beaterDirection;
	if(!_auxIntakeDrawbridge->Pressed()){
		auxBreacherPivot = GetPosition(_auxJoystick, AUX_LEFT_Y_AXIS); // we want Y value to be inverted
		_intake->Pivot(auxBreacherPivot);
	}

	// check intake button to see if its state changed
	beaterDirection = Intake::BeaterBarDirection::STOP;
	if (_auxJoystick->GetRawButton(AUX_INTAKE_IN_BUTTON))
	{
		beaterDirection = Intake::BeaterBarDirection::IN;
	}
	if (GetPosition(_auxJoystick, AUX_INTAKE_OUT_AXIS) > 0)
	{
		beaterDirection = Intake::BeaterBarDirection::OUT;
	}
	_intake->Beater_Bar(beaterDirection);

	// check drawbridge preset
#if 1
	if (_auxIntakeDrawbridge->Process())
	{
		if (_auxIntakeDrawbridge->Pressed())
		{
			_intake->Drawbridge_Position();
		}
		else
		{
			_intake->Stop_Goto();
		}
	}
#endif
}

float Controls::GetPosition(Joystick *joystick, int axis, bool fullrange)
{
	float position;

	position = joystick->GetRawAxis(axis);

	// Deadzone, prevents motors from running
	if ((position > -.3) && (position < .3))
	{
		position = 0; //when not touching the joysticks
	}
	else if (fullrange){
		if(position > 0){
			position = (position - .3)/.7;
		}
		else{
			position = ((abs(position)-.3)/.7)*-1;
		}
	}
	return (position);
}


float Controls::GetPower(float power, bool slow, bool turbo)
{
	// Cube the power to ramp input
	power = power * power * power;

	// Cut power to 3/10 for slower driving
	if (slow)
	{
		power *= .3;
	}
	else if (turbo)
	{
		// Do nothing, keep full power
	}
	else
	{
		// Cut power to 8/10 for normal operation
		power *= .8;
	}
	return (power);
}
float Controls::GetPowerTurn(float power, bool slow, bool turbo)
{
	// Cube the power to ramp input
	power = power * power * power;

	// Cut power to /10 for slower driving
	if (slow)
	{
		power *= .5;
	}
	else if (turbo)
	{
		// Do nothing, keep full power
	}
	else
	{
		// Cut power to 8/10 for normal operation
		power *= .8;
	}
	return (power);
}


// Broken switches
void Controls::ProcessBroken()
{
	if (_brokenBreacherPivot->Process())
	{
		_intake->Pivot_Set_Broken(_brokenBreacherPivot->Pressed());
	}
	if (_brokenBreacherIntake->Process())
	{
		_intake->Beater_Bar_Set_Broken(_brokenBreacherIntake->Pressed());
	}
	// if (_brokenArmRotate->Process())
	// {
	// 	// _scaler->Pivot_Set_Broken(_brokenArmRotate->Pressed());
	// }
	// if (_brokenArmTelescope->Process())
	// {
	// 	// _scaler->Extension_Set_Broken(_brokenArmTelescope->Pressed());
	// }
	// if (_brokenInPitMode->Process())
	// {
	// 	// _scaler->RunInPitMode(_brokenInPitMode->Pressed());
	// 	SmartDashboard::PutNumber("thunderdashboard_inpitmode", (_brokenInPitMode->Pressed() ? 1 : 0));
	// }
	// use tank broken switch is handled in ProcessControllerDriver()
}

// process items for the dashboard
void Controls::ProcessDashboard()
{
	// NOTE: only update items if they changed (or first time in here)

	// see if breacher pivot changed
	int breacherPosition = (int)(_intake->Pivot_Get_Angle() * 100.0);
	if ((breacherPosition != _breacherPosition) || !_dashboardInitialized)
	{
		_breacherPosition = breacherPosition;
		SmartDashboard::PutNumber("thunderdashboard_breacher", _breacherPosition);
	}

	// see if robot has ball
	int haveBall;
	if (_intake->Is_Ball_Acquired())
	{
		haveBall = 1;
	}
	else
	{
		haveBall = 0;
	}
	if ((haveBall != _haveBall) || !_dashboardInitialized)
	{
		_haveBall = haveBall;
		SmartDashboard::PutNumber("thunderdashboard_haveball", _haveBall);
	}

	// get state of drive swap to see if it changed
	if ((_swapDrive != _driveReverse) || !_dashboardInitialized)
	{
		_driveReverse = _swapDrive;
		SmartDashboard::PutNumber("thunderdashboard_drivereverse", _driveReverse);
	}

	// only force the values to go up once
	_dashboardInitialized = true;
}