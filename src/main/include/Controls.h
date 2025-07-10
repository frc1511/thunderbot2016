#pragma once

#include "Drive.h"
#include "Intake.h"
#include "ControlsButton.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

class Controls{

public:
	// Indicates the state of the robot
	typedef enum {TELE_OP, DISABLED} RobotMode;
	// constructor and deconstructor
	Controls(Drive *drive, Intake *intake);
	~Controls();

	// Called in Tele_Op and Disabled and Auto, Broken switch, Xbox controller, Updates dashboard (He is the man for thee job)
	void Process(RobotMode robotmode);// Debug goes in process


private:
	void ProcessBroken();
	void ProcessControllerDriver();
	void ProcessControllerAux();
	float GetPosition(frc::Joystick *joystick, int axis, bool fullrange = false);
	float GetPower(float power, bool slow, bool turbo);
	float GetPowerTurn(float power, bool slow, bool turbo);
	void ProcessDashboard();

	Drive *_drive;
	Intake *_intake;

	// broken switches
	frc::Joystick *_brokenJoystick;
	ControlsButton *_brokenBreacherPivot;
	ControlsButton *_brokenBreacherIntake;
	ControlsButton *_driveDisable; //// _brokenArmRotate
	ControlsButton *_auxDisable; ////_brokenArmTelescope
	ControlsButton *_brokenInPitMode;

	// Joysticks for drivers
	frc::XboxController _driverJoystick {0};
	frc::XboxController _auxJoystick {1};

	// Buttons for drivers
	ControlsButton *_driverSwapDrive;
	int _swapDrive;				// state based on control (i.e. Controls is managing the state)

	// Buttons for aux
	bool _auxIntakeDrawbridge;

	// Dashboard
	bool _dashboardInitialized;
	int _breacherPosition;
	int _haveBall;
	bool _driveDisabled;
	bool _auxDisabled;
	// int _armLocked;
	// int _frontCamera;
	int _driveReverse;
};