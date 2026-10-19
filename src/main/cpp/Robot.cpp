#include "Robot.h"
<<<<<<< HEAD
#include "season/HubShift.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
=======
#include "base/AdvantageScope.h"
>>>>>>> 9800ddb11cf9cf52487c69cadcef013d3169d9bf
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>
Robot::Robot() {
  frc::DataLogManager::Start();
  base::PublishTelemetryMetadata("2026-Rebuilt-cpp");
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
void Robot::RobotPeriodic() {
 frc2::CommandScheduler::GetInstance().Run();
 container_.Periodic();
 auto alliance=frc::DriverStation::GetAlliance();
 std::optional<season::Alliance> own;
 if(alliance)own=*alliance==frc::DriverStation::Alliance::kBlue?season::Alliance::Blue:season::Alliance::Red;
 auto info=season::ComputeHubShift(frc::DriverStation::GetMatchTime().value(),frc::DriverStation::IsTeleop(),own,frc::DriverStation::GetGameSpecificMessage());
 int seconds=static_cast<int>(info.matchTime);
 frc::SmartDashboard::PutString("Match Time",fmt::format("{:02d}:{:02d}.{}",seconds/60,seconds%60,static_cast<int>(info.matchTime*10)%10));
 frc::SmartDashboard::PutString("Shifts/Remaining Shift Time",fmt::format("{:.1f}",info.remainingTime));
 frc::SmartDashboard::PutBoolean("Shifts/Shift Active",info.active);
 frc::SmartDashboard::PutString("Shifts/Game State",frc::DriverStation::IsAutonomous()?"Autonomous":season::ShiftStateName(info.currentShift));
 frc::SmartDashboard::PutBoolean("Shifts/Active First?",own.value_or(season::Alliance::Blue)==season::FirstActiveAlliance(frc::DriverStation::GetGameSpecificMessage()));
 frc::SmartDashboard::PutString("Shifts/Match Time Color",info.matchTimeColor);
 frc::SmartDashboard::PutString("Shifts/Shift Time Color",info.shiftTimeColor);
}
void Robot::AutonomousInit() { container_.CancelChecks(); container_.ResetSimulation(); autonomous_=container_.GetAutonomousCommand(); if(autonomous_) autonomous_->Schedule(); }
void Robot::TeleopInit() { container_.CancelChecks(); if(autonomous_) autonomous_->Cancel(); }
void Robot::TestInit() { container_.CancelChecks(); frc2::CommandScheduler::GetInstance().CancelAll(); }
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

void Robot::SimulationPeriodic(){container_.SimulationPeriodic();}
void Robot::DisabledInit(){container_.CancelChecks();}
