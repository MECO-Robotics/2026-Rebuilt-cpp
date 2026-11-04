#include "Robot.h"
#include "season/RobotControlConfig.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/detail/RobotPlatform.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <base/season2026/MatchContext.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <units/time.h>
#include <frc2/command/CommandScheduler.h>
// Configure platform services before the robot loop starts.
Robot::Robot() : frc::TimedRobot{season::hardware::kControlPeriod}
{
  lifecycle_.Initialize();
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(season::hardware::kBrownoutVoltage);
  frc2::CommandScheduler::GetInstance().SetPeriod(season::hardware::kControlPeriod);
}
// Refresh match context before running scheduled commands.
void Robot::RobotPeriodic()
{
  const auto match = base::season2026::ReadMatchContext();
  container_.SetMatchContext(match);
  container_.ControlMechanisms();
  if (autonomousPending_ && frc::DriverStation::IsAutonomousEnabled() && container_.AutonomousReady())
  {
    autonomousPending_ = false;
    if (autonomous_)
      container_.ScheduleAutonomous(autonomous_);
  }
  lifecycle_.BeforeScheduler();
  frc2::CommandScheduler::GetInstance().Run();
  lifecycle_.AfterScheduler();
  auto nt = nt::NetworkTableInstance::GetDefault().GetTable("/Match");
  nt->GetEntry("Phase").SetString(match.phase);
  nt->GetEntry("TimeKnown").SetBoolean(match.periodTime.has_value());
  nt->GetEntry("Time").SetDouble(match.periodTime.value_or(-1));
  nt->GetEntry("GameDataValid").SetBoolean(match.gameDataValid);
  nt->GetEntry("GameData").SetString(match.gameData);
  nt->GetEntry("InactiveFirstAlliance").SetString(base::season2026::AllianceLabel(match.inactiveFirstAlliance));
  nt->GetEntry("FirstActiveAlliance").SetString(base::season2026::AllianceLabel(match.firstActiveAlliance));
  nt->GetEntry("HubNextChangeKnown").SetBoolean(match.ownNextChange.has_value());
  nt->GetEntry("HubNextChangeSeconds").SetDouble(match.ownNextChange.value_or(-1));
  nt->GetEntry("HubKnown").SetBoolean(match.ownActive.has_value());
  nt->GetEntry("HubActive").SetBoolean(match.ownActive.value_or(false));
  nt->GetEntry("BlueHubKnown").SetBoolean(match.blueActive.has_value());
  nt->GetEntry("RedHubKnown").SetBoolean(match.redActive.has_value());
  nt->GetEntry("BlueHubActive").SetBoolean(match.blueActive.value_or(false));
  nt->GetEntry("RedHubActive").SetBoolean(match.redActive.value_or(false));
  nt->GetEntry("BlueNextChange").SetDouble(match.blueNextChange.value_or(-1));
  nt->GetEntry("RedNextChange").SetDouble(match.redNextChange.value_or(-1));
  frc::SmartDashboard::PutString("Shifts/Game State", match.phase);
  frc::SmartDashboard::PutBoolean("Shifts/Shift Active", match.ownActive.value_or(false));
  frc::SmartDashboard::PutBoolean("Shifts/State Known", match.ownActive.has_value());
  if (match.periodTime)
  {
    const int seconds = static_cast<int>(*match.periodTime);
    frc::SmartDashboard::PutString("Match Time", fmt::format("{:02d}:{:02d}.{}", seconds / 60, seconds % 60,
                                                             static_cast<int>(*match.periodTime * 10) % 10));
  }
  else
    frc::SmartDashboard::PutString("Match Time", "unknown");
  const auto next = match.alliance ? (*match.alliance == frc::DriverStation::Alliance::kBlue ? match.blueNextChange
                                                                                             : match.redNextChange)
                                   : std::nullopt;
  frc::SmartDashboard::PutString("Shifts/Remaining Shift Time", next ? fmt::format("{:.1f}", *next) : "unknown");
  frc::SmartDashboard::PutBoolean(
      "Shifts/Active First?",
      match.alliance && match.gameDataValid &&
          ((*match.alliance == frc::DriverStation::Alliance::kBlue) == (match.gameData == "R")));
}
// Prepare platform feedback before scheduling the selected autonomous command.
void Robot::AutonomousInit()
{
  container_.CancelChecks();
  // Scheduling initializes AutoDrive immediately, before RobotPeriodic runs.
  // Supply this packet's context for initialization as well as execution.
  container_.SetMatchContext(base::season2026::ReadMatchContext());
  container_.PrepareAutonomous();
  autonomous_ = container_.GetAutonomousCommand();
  autonomousPending_ = autonomous_ != nullptr;
  if (autonomousPending_ && container_.AutonomousReady())
  {
    autonomousPending_ = false;
    container_.ScheduleAutonomous(autonomous_);
  }
}
// Release autonomous ownership before accepting driver input.
void Robot::TeleopInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
  if (autonomous_)
    autonomous_->Cancel();
}
// Test mode starts with no scheduled robot commands.
void Robot::TestInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
  frc2::CommandScheduler::GetInstance().CancelAll();
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

// Advance platform services around the scene update.
void Robot::SimulationPeriodic()
{
  lifecycle_.BeforeScene();
  container_.SimulationPeriodic();
  lifecycle_.AfterScene();
}
void Robot::DisabledInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
}
