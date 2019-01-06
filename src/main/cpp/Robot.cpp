#include "frc/WPILib.h"
#include "Drive.h"
#include "Intake.h"
#include <math.h>
#include "Scaler.h"
#include "Feedback.h"
#include "Controls.h"
#include "Autonomous.h"

class ThunderBot : public SampleRobot
{
	Intake intake;
	Feedback feedback;
	Drive drive;
	Scaler scaler;
	Controls controls;
	Auto autonomous;

private:

	const double MAIN_LOOP_WAIT_TIME = 0.05;
	int tank = 1;
	float speed = 0;

public:


	ThunderBot():
		intake(),feedback(),drive(),scaler(&intake),
		controls(&drive,&intake,&scaler,&feedback),
		autonomous(&drive,&intake)
	{
		feedback.initCameras();
	}
	void Disabled()
	{
		autonomous.AutoDashboardSend();
		drive.CalibrateGyro();


		while (IsDisabled()) {
			controls.Process(controls.DISABLED);
			intake.Debug(&feedback);
			scaler.Debug(&feedback);
			drive.Debug(&feedback);

			Wait(MAIN_LOOP_WAIT_TIME);
		}
	}

	void Autonomous()
	{
		double autoMode = SmartDashboard::GetNumber("Auto_Mode", 0);	// use default auto mode if not set
		autonomous.SetAutoMode((Auto::AutoMode)autoMode);

		drive.Reset();
		intake.Reset();
		scaler.Reset();
		autonomous.Reset();
		while (IsAutonomous() && IsEnabled()) {
			controls.Process(controls.AUTO);
			scaler.Process();
			drive.Process();
			intake.Process();
			autonomous.Process();
			intake.Debug(&feedback);
			scaler.Debug(&feedback);
			drive.Debug(&feedback);

			Wait(MAIN_LOOP_WAIT_TIME);
		}
		intake.Stop_Goto();
	}

	void OperatorControl()
	{
		drive.Reset();
		intake.Reset();
		scaler.Reset();

		while (IsOperatorControl() && IsEnabled()) {
			 controls.Process(controls.TELE_OP);
			 scaler.Process();
			 drive.Process();
			 intake.Process();
             intake.Debug(&feedback);
             scaler.Debug(&feedback);
             drive.Debug(&feedback);

             Wait(MAIN_LOOP_WAIT_TIME);		// wait for a motor update time
		}
	}

	void Test()
	{
		while (IsTest() && IsEnabled()) {
			Wait(MAIN_LOOP_WAIT_TIME);
		}
	}
};

START_ROBOT_CLASS(ThunderBot);
