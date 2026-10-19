#include "Robot.h"
#include "base/AdvantageScope.h"
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>
Robot::Robot() {
  frc::DataLogManager::Start();
  base::PublishTelemetryMetadata("2026_Base_Bot-cpp");
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }
void Robot::AutonomousInit() { autonomous_=container_.GetAutonomousCommand(); if(autonomous_) autonomous_->Schedule(); }
void Robot::TeleopInit() { if(autonomous_) autonomous_->Cancel(); }
void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
