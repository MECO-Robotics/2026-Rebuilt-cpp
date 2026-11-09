#include "Robot.h"
#include "season/RobotControlConfig.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/hardware/RobotPlatform.h"
#include <base/TelemetryPublishers.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include "season/runtime/MatchContext.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <units/time.h>
#include <frc2/command/CommandScheduler.h>
#include <base/Replay.h>
#include <base/AdvantageKitLog.h>
// Configure platform services before the robot loop starts.
Robot::Robot() : frc::TimedRobot{season::hardware::kControlPeriod}
{
  if (!base::replay::Active())
    base::logging::StartRobotDataLogSources();
  frc::RobotController::SetBrownoutVoltage(season::hardware::kBrownoutVoltage);
  frc2::CommandScheduler::GetInstance().SetPeriod(season::hardware::kControlPeriod);
}
Robot::~Robot()
{
  base::replay::ShutdownRecording();
  base::logging::StopNetworkTablesDataLog();
  base::logging::StopAdvantageKitLog();
}
// Refresh match context before running scheduled commands.
void Robot::RobotPeriodic()
{
  base::replay::RunFrame([this] { RunRobotPeriodic(); });
}
void Robot::RunRobotPeriodic()
{
  const auto match = season::runtime::ReadMatchContext();
  container_.SetMatchContext(match);
  container_.ControlMechanisms();
  if (autonomousPending_ && match.enabled && match.autonomous && container_.AutonomousReady())
  {
    autonomousPending_ = false;
    if (autonomous_)
      container_.ScheduleAutonomous(autonomous_);
  }
  container_.BeforeScheduler();
  frc2::CommandScheduler::GetInstance().Run();
  container_.AfterScheduler();
  base::telemetry::PostNetworkString("/Match/Phase", match.phase);
  base::telemetry::PostNetworkBoolean("/Match/TimeKnown", match.periodTime.has_value());
  base::telemetry::PostNetworkNumber("/Match/Time", match.periodTime.value_or(-1));
  base::telemetry::PostNetworkBoolean("/Match/GameDataValid", match.gameDataValid);
  base::telemetry::PostNetworkBoolean("/Match/GameDataAvailable", match.gameDataAvailable);
  base::telemetry::PostNetworkBoolean("/Match/PolicyInputsKnown", match.matchPolicyInputsKnown);
  base::telemetry::PostNetworkString("/Match/GameData", match.gameData);
  base::telemetry::PostNetworkString("/Match/InactiveFirstAlliance", season::runtime::AllianceLabel(match.inactiveFirstAlliance));
  base::telemetry::PostNetworkString("/Match/FirstActiveAlliance", season::runtime::AllianceLabel(match.firstActiveAlliance));
  base::telemetry::PostNetworkBoolean("/Match/HubNextChangeKnown", match.ownNextChange.has_value());
  base::telemetry::PostNetworkNumber("/Match/HubNextChangeSeconds", match.ownNextChange.value_or(-1));
  base::telemetry::PostNetworkBoolean("/Match/HubKnown", match.ownActive.has_value());
  base::telemetry::PostNetworkBoolean("/Match/HubActive", match.ownActive.value_or(false));
  base::telemetry::PostNetworkBoolean("/Match/BlueHubKnown", match.blueActive.has_value());
  base::telemetry::PostNetworkBoolean("/Match/RedHubKnown", match.redActive.has_value());
  base::telemetry::PostNetworkBoolean("/Match/BlueHubActive", match.blueActive.value_or(false));
  base::telemetry::PostNetworkBoolean("/Match/RedHubActive", match.redActive.value_or(false));
  base::telemetry::PostNetworkNumber("/Match/BlueNextChange", match.blueNextChange.value_or(-1));
  base::telemetry::PostNetworkNumber("/Match/RedNextChange", match.redNextChange.value_or(-1));
  base::telemetry::PostDashboardString("Shifts/Game State", match.phase);
  base::telemetry::PostDashboardBoolean("Shifts/Shift Active", match.ownActive.value_or(false));
  base::telemetry::PostDashboardBoolean("Shifts/State Known", match.ownActive.has_value());
  base::telemetry::PostDashboardString("Shifts/Policy Inputs", match.matchPolicyInputsKnown ? "known" : "missing");
  base::replay::RecordOutput("Season/Match/PolicyInputs",
                             wpi::json{{"known", match.matchPolicyInputsKnown},
                                       {"gameDataAvailable", match.gameDataAvailable},
                                       {"gameDataValid", match.gameDataValid},
                                       {"periodTimeAvailable", match.periodTime.has_value()},
                                       {"allianceAvailable", match.alliance.has_value()}});
  if (match.periodTime)
  {
    const int seconds = static_cast<int>(*match.periodTime);
    base::telemetry::PostDashboardString("Match Time", fmt::format("{:02d}:{:02d}.{}", seconds / 60, seconds % 60,
                                                             static_cast<int>(*match.periodTime * 10) % 10));
  }
  else
    base::telemetry::PostDashboardString("Match Time", "unknown");
  const auto next = match.alliance ? (*match.alliance == frc::DriverStation::Alliance::kBlue ? match.blueNextChange
                                                                                             : match.redNextChange)
                                   : std::nullopt;
  base::telemetry::PostDashboardString("Shifts/Remaining Shift Time", next ? fmt::format("{:.1f}", *next) : "unknown");
  base::telemetry::PostDashboardBoolean(
      "Shifts/Active First?",
      match.alliance && match.gameDataValid &&
          ((*match.alliance == frc::DriverStation::Alliance::kBlue) == (match.gameData == "R")));
}
// Prepare platform feedback before scheduling the selected autonomous command.
void Robot::AutonomousInit()
{
  base::replay::RunFrame([this] {
    container_.CancelChecks();
    // Scheduling initializes AutoDrive immediately, before RobotPeriodic runs.
    // Supply this packet's context for initialization as well as execution.
    container_.SetMatchContext(season::runtime::ReadMatchContext());
    container_.PrepareAutonomous();
    autonomous_ = container_.GetAutonomousCommand();
    autonomousPending_ = autonomous_ != nullptr;
    if (autonomousPending_ && container_.AutonomousReady())
    {
      autonomousPending_ = false;
      container_.ScheduleAutonomous(autonomous_);
    }
  });
}
// Release autonomous ownership before accepting driver input.
void Robot::TeleopInit()
{
  base::replay::RunFrame([this] {
    container_.SetMatchContext(season::runtime::ReadMatchContext());
    autonomousPending_ = false;
    container_.CancelChecks();
    if (autonomous_)
      autonomous_->Cancel();
  });
}
// Test mode starts with no scheduled robot commands.
void Robot::TestInit()
{
  base::replay::RunFrame([this] {
    container_.SetMatchContext(season::runtime::ReadMatchContext());
    autonomousPending_ = false;
    container_.CancelChecks();
    frc2::CommandScheduler::GetInstance().CancelAll();
  });
}
#ifndef RUNNING_FRC_TESTS
int main()
{
  base::replay::InitializeFromEnvironment();
  if (base::replay::Active())
  {
    Robot robot;
    base::replay::RunReplay([&robot] { robot.RobotInit(); }, [&robot] { robot.ReplayLoop(); });
    base::replay::FlushOutput();
    return 0;
  }
  return frc::StartRobot<Robot>();
}
#endif

// Advance platform services around the scene update.
void Robot::SimulationPeriodic()
{
  container_.BeforeScene();
  container_.SimulationPeriodic();
  container_.AfterScene();
}
void Robot::DisabledInit()
{
  base::replay::RunFrame([this] {
    container_.SetMatchContext(season::runtime::ReadMatchContext());
    autonomousPending_ = false;
    container_.CancelChecks();
  });
}
