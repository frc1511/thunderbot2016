#ifndef CONTROLS_H
#define CONTROLS_H

#include "Feedback.h"
#include "Drive.h"
#include "Intake.h"
#include "Scaler.h"
#include "ControlsButton.h"

using namespace frc;

class Controls{

public:
	// Indicates the state of the robot
	typedef enum {AUTO, TELE_OP, DISABLED} RobotMode;
	// constructor and deconstructor
	Controls(Drive *drive, Intake *intake, Scaler *scaler, Feedback *feedback);
	~Controls();

	// Called in Tele_Op and Disabled and Auto, Broken switch, Xbox controller, Updates dashboard (He is the man for thee job)
	void Process(RobotMode robotmode);// Debug goes in process


private:
	void ProcessBroken();
	void ProcessControllerDriver();
	void ProcessControllerAux();
	float GetPosition(Joystick *joystick, int axis, bool fullrange = false);
	float GetPower(float power, bool slow, bool turbo);
	float GetPowerTurn(float power, bool slow, bool turbo);
	void ProcessDashboard();

	Drive *_drive;
	Intake *_intake;
	Scaler *_scaler;
	Feedback *_feedback;

	// broken switches
	Joystick *_brokenJoystick;
	ControlsButton *_brokenBreacherPivot;
	ControlsButton *_brokenBreacherIntake;
	ControlsButton *_brokenArmRotate;
	ControlsButton *_brokenArmTelescope;
	ControlsButton *_brokenInPitMode;
	// do not need one for borken use tank


	// Joysticks for drivers
	Joystick *_driverJoystick;
	Joystick *_auxJoystick;

	// Buttons for drivers
	ControlsButton *_driverSwapDrive;
	ControlsButton *_driverSwapCameras;
	int _swapDrive;				// state based on control (i.e. Controls is managing the state)

	// Buttons for aux
	ControlsButton *_auxSwapCameras;
	ControlsButton *_auxScalerUp;
	ControlsButton *_auxScalerDown;
	ControlsButton *_auxIntakeDrawbridge;

	// Dashboard
	bool _dashboardInitialized;
	int _breacherPosition;
	int _scalerPivot;
	int _scalerExt;
	int _haveBall;
	int _armLocked;
	int _frontCamera;
	int _driveReverse;
};

#endif // CONTROLS_H
