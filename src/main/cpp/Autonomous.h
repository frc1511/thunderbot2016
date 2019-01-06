#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "Intake.h"
#include "Drive.h"
#include "timer.h"
#include "Feedback.h"
using namespace frc;
//#include "lights.h"

class Auto {
public:
	typedef enum {
			kAutoNothing,
			kAutoReach,
			kAutoTerrainBreach,
			kAutoRockWall,
			kAutoGoUnderLowBar,
			kAutoLowBarBackandAgain,
			kAutoLowBarLowScore,
			kAutoLowBarLowScoreNot,
			kAutoFrenchKiss,
			kAutoMoatLowScore
		}AutoMode;
		Auto (Drive*D, Intake*I);

	void AutoDashboardSend();

	void SetAutoMode(AutoMode Mode);

	void Process();

	void AutoDoNothing();

	void AutoReach();

	void AutoTerrainBreach();

	void AutoRockWall();

	void AutoGoUnderLowBar();

	void AutoLowBarBackandAgain();

	void AutoLowBarLowScore(bool spitBall);

	void AutoFrenchKiss();

	void AutoMoatLowScore(bool spitBall);

	void Reset();

	void Debug(Feedback* Feedback);
private:

	AutoMode Mode;
	Drive*drive;
	Intake*intake;

	int currentStep;

};

#endif // AUTONOMOUS_H_
