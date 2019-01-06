#include "Autonomous.h"
#include "frc/WPILib.h"

	Auto::Auto(Drive*D, Intake*I){
		drive = D;
		intake = I;
		Mode = kAutoNothing;
		currentStep = 1;
	}
	void Auto::SetAutoMode(AutoMode AMode){
		Mode = AMode;
		printf("Auto mode is %d\n", AMode);
	}
	void Auto::Process(){

	switch(Mode){
		case kAutoNothing:
			AutoDoNothing();
			break;
		case kAutoReach:
			AutoReach();
			break;
		case kAutoTerrainBreach:
			AutoTerrainBreach();
			break;
		case kAutoRockWall:
			AutoRockWall();
			break;
		case kAutoGoUnderLowBar:
			AutoGoUnderLowBar();
			break;
		case kAutoLowBarBackandAgain:
			AutoLowBarBackandAgain();
			break;
		case kAutoLowBarLowScore:
			AutoLowBarLowScore(true);
			break;
		case kAutoLowBarLowScoreNot:
			AutoLowBarLowScore(false);
			break;
		case kAutoFrenchKiss:
			AutoFrenchKiss();
			break;
		case kAutoMoatLowScore:
			AutoMoatLowScore(true);
			break;


		}
	}
	void Auto::AutoDashboardSend(){
		char buffer[256] = "";
		char key[32];

		sprintf(&buffer[strlen(buffer)],",%d", kAutoNothing);
		sprintf(key,"thunderdashboard_auto_%d", kAutoNothing);
		SmartDashboard::PutString(key, "Does nothing....Duh");

		sprintf(&buffer[strlen(buffer)],",%d", kAutoReach);
		sprintf(key,"thunderdashboard_auto_%d", kAutoReach);
		SmartDashboard::PutString(key, "Reaches the Defenses");

		sprintf(&buffer[strlen(buffer)],",%d", kAutoTerrainBreach);
		sprintf(key,"thunderdashboard_auto_%d", kAutoTerrainBreach);
		SmartDashboard::PutString(key, "Goes over the Moat, Uneven terrain, or Ramparts");

		sprintf(&buffer[strlen(buffer)],",%d", kAutoRockWall);
		sprintf(key,"thunderdashboard_auto_%d", kAutoRockWall);
		SmartDashboard::PutString(key, "Goes over the Rockwall");


		sprintf(&buffer[strlen(buffer)],",%d", kAutoGoUnderLowBar);
		sprintf(key,"thunderdashboard_auto_%d", kAutoGoUnderLowBar);
		SmartDashboard::PutString(key, "Goes under the Lowbar");


#if 0
		sprintf(&buffer[strlen(buffer)],",%d", kAutoLowBarBackandAgain);
		sprintf(key,"thunderdashboard_auto_%d", kAutoLowBarBackandAgain);
		SmartDashboard::PutString(key, "Goes under low bar and comes back into neutral zone");
#endif


		sprintf(&buffer[strlen(buffer)],",%d", kAutoLowBarLowScore);
		sprintf(key,"thunderdashboard_auto_%d", kAutoLowBarLowScore);
		SmartDashboard::PutString(key, "Goes under Lowbar and scores a low goal");


		sprintf(&buffer[strlen(buffer)],",%d", kAutoLowBarLowScoreNot);
		sprintf(key,"thunderdashboard_auto_%d", kAutoLowBarLowScoreNot);
		SmartDashboard::PutString(key, "Goes under Lowbar and just drives to low goal");



		sprintf(&buffer[strlen(buffer)],",%d", kAutoFrenchKiss);
		sprintf(key,"thunderdashboard_auto_%d", kAutoFrenchKiss);
		SmartDashboard::PutString(key, "Goes over the Cheval de frise");

		sprintf(&buffer[strlen(buffer)],",%d", kAutoMoatLowScore);
		sprintf(key,"thunderdashboard_auto_%d", kAutoMoatLowScore);
		SmartDashboard::PutString(key, "Goes over moat and scores (hopefully)");


		SmartDashboard::PutString("thunderdashboard_auto_list", buffer);
	}
	void Auto::AutoDoNothing(){

	}
	void Auto::AutoReach(){
		if(currentStep == 1){
			drive->Auto_Straight(.5,50);
			currentStep ++;
		}
		if(currentStep == 2){
			if(drive->Auto_Move_Complete() == true){
			}
		}
	}
	void Auto::AutoTerrainBreach(){
		if(currentStep == 1){
			drive->Auto_Straight(.7,200);
			currentStep ++;
		}
		if(currentStep == 2){
			if(drive->Auto_Move_Complete() == true){
			}
		}
	}
	void Auto::AutoRockWall(){
		if(currentStep == 1){
			drive->Auto_Straight(.75,175);
			currentStep ++;
		}
		if(currentStep == 2){
			if(drive->Auto_Move_Complete() == true){
			}
		}
	}
	void Auto::AutoGoUnderLowBar(){
		if(currentStep == 1){
			intake->Pivot_Go_To(0);
			currentStep ++;
		}
		if(currentStep == 2){
			if(intake->At_Pivot_Go_To() == true){
				//intake->Pivot_Go_To(0);
				currentStep ++;
			}
		}
		if(currentStep == 3){
			drive->Auto_Straight(.7, 125);
			//intake->Pivot_Go_To(0);
			currentStep ++;
		}
		if(currentStep == 4){
			if(drive->Auto_Move_Complete() == true){
				//intake->Pivot_Go_To(0);
				currentStep++;
			}
		}
		if(currentStep == 5){
			intake->Stop_Goto();
		}
	}
	void Auto::AutoLowBarBackandAgain(){
		if(currentStep == 1){
			intake->Pivot_Go_To(.2);
			currentStep ++;
		}
		if(currentStep == 2){
			if(intake->At_Pivot_Go_To() == true)
				currentStep ++;
		}
		if(currentStep == 3){
			drive->Auto_Straight(.5, 50);
			currentStep ++;
		}
		if(currentStep == 4){
			if(drive->Auto_Move_Complete() == true)
				currentStep ++;
		}
		if(currentStep == 5){
			drive->Auto_Straight(.75, -50);
			currentStep ++;
		}
		if(currentStep == 6){
			if(drive->Auto_Move_Complete() == true){
			}
		}
	}
	void Auto::AutoLowBarLowScore(bool spitBall){
		if(currentStep == 1){					//puts the breacher down
					intake->Pivot_Go_To(0);
					currentStep ++;
			}
			if(currentStep == 2){				//makes sure the breacher is actually down before...
				if(intake->At_Pivot_Go_To() == true){
					//intake->Pivot_Go_To(0);
					currentStep ++;
				}
			}
			if(currentStep == 3){					//drives straight at .6 for 128 inches
				drive->Auto_Straight(.7, 128);	//formerly 118
				//intake->Pivot_Go_To(0);
				currentStep ++;
			}
			if(currentStep == 4){					//when it's done moving forwards
				if(drive->Auto_Move_Complete() == true){
					//intake->Pivot_Go_To(0);
					currentStep++;
				}
			}
			if(currentStep == 5){					//sets the breacher to a little bit up
				intake->Pivot_Go_To(1);				//formerly .25 but 1 for testing
				currentStep ++;
			}
			if(currentStep == 6){					//waits until it gets there
				if(intake->At_Pivot_Go_To() == true){
					//intake->Pivot_Go_To(0);
			currentStep ++;
				}
			}
			if(currentStep == 7){			//turns while printing "the great turn is underway"
				drive->Auto_Turn(53);
				currentStep++;
			}
			if(currentStep == 8){			//waits until the great turn is finished
				if (drive->Auto_Move_Complete()){
					printf("Kiss is complete \n");
					//currentStep++;
					currentStep = 11;
				}
			}
			if(currentStep == 9){			//drives straight 60in at .7 speed       TOOK THIS OUT
				drive->Auto_Straight(.7, 60);
				//intake->Pivot_Go_To(0);
				currentStep ++;
			}
			if(currentStep == 10){			//waits until finishes going straight    TOOK THIS OUT
				if(drive->Auto_Move_Complete() == true){
					//intake->Pivot_Go_To(0);
					currentStep++;
				}
			}
			if(currentStep == 11){
				intake->Pivot_Go_To(.25);				//moves the breacher to the height for score NEW
				currentStep ++;
			}
			if(currentStep == 12){						//waits until breacher in correct position   NEW
				if(intake->At_Pivot_Go_To() == true){
					//intake->Pivot_Go_To(0);
					currentStep ++;
				}
			}
			if(currentStep == 13){			//puts the ball out hopefully into goal
				if (spitBall){
					intake->Beater_Bar(Intake::BeaterBarDirection::OUT);
				}
				currentStep++;
			}
			if(currentStep == 14){			//stops the goto so it works in teleop
				intake->Stop_Goto();
			}
		}
	void Auto::AutoFrenchKiss(){
		if(currentStep == 1){
			printf("I have started the Kiss \n");
			drive->Auto_Turn(90);
			currentStep++;
		}
		if(currentStep == 2){
			if (drive->Auto_Move_Complete()){
				printf("Kiss is complete \n");
				currentStep++;
			}
		}
	}
	void Auto::AutoMoatLowScore(bool spitBall){
		if(currentStep == 1){					//drives straight at .6 for 180 inches
			drive->Auto_Straight(.6    , 180);
			//intake->Pivot_Go_To(0);
			currentStep ++;
		}
		if(currentStep == 2){					//when it's done moving forwards
			if(drive->Auto_Move_Complete() == true){
				//intake->Pivot_Go_To(0);
				currentStep++;
			}
		}
		if(currentStep == 3){			//turns while printing "the other great turn is underway"
			drive->Auto_Turn_Moat(-35);
			currentStep++;
		}
		if(currentStep == 4){			//waits until the great turn is finished
			if (drive->Auto_Move_Complete()){
				printf("Kiss is complete \n");
				//currentStep++;
				currentStep = 7;
			}
		}
		if(currentStep == 5){			//drives straight 60in at .7 speed       TOOK THIS OUT
			drive->Auto_Straight(.7, 60);
			//intake->Pivot_Go_To(0);
			currentStep ++;
		}
		if(currentStep == 6){			//waits until finishes going straight    TOOK THIS OUT
			if(drive->Auto_Move_Complete() == true){
				//intake->Pivot_Go_To(0);
				currentStep++;
			}
		}
		if(currentStep == 7){
			intake->Pivot_Go_To(.25);				//moves the breacher to the height for score NEW
			currentStep ++;
		}
		if(currentStep == 8){						//waits until breacher in correct position   NEW
			if(intake->At_Pivot_Go_To() == true){
				//intake->Pivot_Go_To(0);
				currentStep ++;
			}
		}
		if(currentStep == 9){			//puts the ball out hopefully into goal
			if (spitBall){
				intake->Beater_Bar(Intake::BeaterBarDirection::OUT);
			}
			currentStep++;
		}
		if(currentStep == 10){			//stops the goto so it works in teleop
			intake->Stop_Goto();
		}
	}
	void Auto::Reset(){
		currentStep = 1;
		intake->Stop_Goto();
	}
	void Auto::Debug(Feedback* Feedback){

	}
