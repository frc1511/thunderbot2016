// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  drive.Reset();
	intake.Reset();
}
void Robot::TeleopPeriodic() {
  controls.Process(controls.TELE_OP);
  drive.Process();
  intake.Process();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {
  controls.Process(controls.DISABLED);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif