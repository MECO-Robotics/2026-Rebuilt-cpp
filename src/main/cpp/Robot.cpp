#include "Robot.h"
#include "season/drive/FastControlProfile.h"
#include "season/simulation/RuntimeConfig.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/MechanismConfig.h"
#include "season/mechanisms/RobotPlatform.h"
#include <string_view>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include "base/SimulationLatency.h"
#include <base/season2026/MatchContext.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <cstdlib>
#include <units/time.h>
#include <stdexcept>
#include <array>
#include <algorithm>
#include <cmath>
#ifndef __FRC_ROBORIO__
#include "base/SimulationRuntime.h"
#include "meco/sim/BatterySupply.h"
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <charconv>
#include <cmath>
#endif
#include <frc2/command/CommandScheduler.h>
// Construct simulation services before the robot loop starts.
Robot::Robot()
{
#ifndef __FRC_ROBORIO__
  if (const char *text = std::getenv("FRC_SIM_NT_PORT"))
  {
    const std::string_view value{text};
    unsigned int port = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), port);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || port < 1024 || port > 65535)
      throw std::invalid_argument("FRC_SIM_NT_PORT must be an integer in [1024,65535]");
    auto nt = nt::NetworkTableInstance::GetDefault();
    nt.StopServer();
    nt.StartServer("", "127.0.0.1", 0, port);
  }
#endif
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation())
    practice_ = std::make_unique<meco::sim::PracticeMatchController>(meco::sim::PracticeMatchProfile{season::simulation::runtimeconfig::kPracticeAutonomousSeconds, season::simulation::runtimeconfig::kPracticeAssessmentSeconds, season::simulation::runtimeconfig::kPracticeTeleopSeconds});
#endif
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(season::hardware::kBrownoutVoltage);
  fastControlEnabled_ = season::hardware::FastControlEnabled();
  frc2::CommandScheduler::GetInstance().SetPeriod(units::second_t{
      fastControlEnabled_ ? season::drive::fastcontrol::Config().periodMs / 1000.0 : season::hardware::kControlPeriod.value()});
  if (fastControlEnabled_)
  {
    fastControlSamplesPublisher_ = nt::NetworkTableInstance::GetDefault()
                                       .GetDoubleArrayTopic("/Simulation/FastControl/Samples")
                                       .Publish();
    AddPeriodic([this]
                { FastControlPeriodic(); },
                units::second_t{season::drive::fastcontrol::Config().periodMs / 1000.0});
  }
}
// Refresh match context before running scheduled commands.
void Robot::RobotPeriodic()
{
  const auto match = base::season2026::ReadMatchContext();
  container_.SetMatchContext(match);
  // Baseline mechanism control is 20 ms; the optimized callback owns its faster ticks.
  container_.ControlMechanisms20ms();
  if (autonomousPending_ && frc::DriverStation::IsAutonomousEnabled() && container_.SimulationResetReady())
  {
    autonomousPending_ = false;
    if (autonomous_)
      container_.ScheduleAutonomous(autonomous_);
  }
  if (!fastControlEnabled_)
  {
    base::simulation::BeginLatencyProbe();
    frc2::CommandScheduler::GetInstance().Run();
    base::simulation::EndLatencyProbe();
  }
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
// Run command decisions faster while TimedRobot and physics stay at 20 ms.
void Robot::FastControlPeriodic()
{
  if (!fastControlEnabled_)
    return;
  const int64_t periodUs = static_cast<int64_t>(season::drive::fastcontrol::Config().periodMs * 1000.0);
  const int64_t startUs = static_cast<int64_t>(frc::RobotController::GetFPGATime());
  if (nextFastControlFpgaUs_ == 0)
    nextFastControlFpgaUs_ = startUs;
  const int64_t scheduledUs = nextFastControlFpgaUs_;
  const int64_t latenessUs = std::max<int64_t>(0, startUs - scheduledUs);
  nextFastControlFpgaUs_ = scheduledUs + periodUs;
  if (latenessUs >= periodUs)
    nextFastControlFpgaUs_ += (latenessUs / periodUs) * periodUs;

  frc::DriverStation::RefreshData();
  base::simulation::BeginLatencyProbe();
  frc2::CommandScheduler::GetInstance().Run();
  container_.ControlMechanismsFast();
  base::simulation::EndLatencyProbe();
  const int64_t finishUs = static_cast<int64_t>(frc::RobotController::GetFPGATime());
  const std::array<double, 7> sample{static_cast<double>(fastControlSequence_++),
                                     static_cast<double>(scheduledUs),
                                     static_cast<double>(startUs),
                                     static_cast<double>(finishUs),
                                     static_cast<double>(finishUs - startUs),
                                     static_cast<double>(latenessUs),
                                     frc::DriverStation::IsEnabled() ? 1.0 : 0.0};
  fastControlSamplesPublisher_.Set(sample, nt::Now());
}
// Reset the scene before scheduling the selected autonomous command.
void Robot::AutonomousInit()
{
  container_.CancelChecks();
  // Scheduling initializes AutoDrive immediately, before RobotPeriodic runs.
  // Supply this packet's context for initialization as well as execution.
  container_.SetMatchContext(base::season2026::ReadMatchContext());
  container_.ResetSimulation();
  autonomous_ = container_.GetAutonomousCommand();
  autonomousPending_ = autonomous_ != nullptr;
  if (autonomousPending_ && container_.SimulationResetReady())
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

// Advance the practice controller, scene, battery, and simulated DS packet.
void Robot::SimulationPeriodic()
{
#ifndef __FRC_ROBORIO__
  if (practice_)
    practice_->Update();
#endif
  container_.SimulationPeriodic();
#ifndef __FRC_ROBORIO__
  auto &runtime = base::simulation::GetSimulationRuntime();
  static auto physicsClockPublisher = nt::NetworkTableInstance::GetDefault()
                                          .GetDoubleArrayTopic("/Simulation/Physics/Samples")
                                          .Publish();
  physicsClockPublisher.Set(std::array<double, 3>{static_cast<double>(nt::Now()),
                                                  static_cast<double>(frc::RobotController::GetFPGATime()), runtime.Time()});
  // Coulomb-count an estimated 18 Ah FRC battery for the life of this sim
  // process. OCV falls with state of charge; the shared supply model then
  // adds the configured internal-resistance sag under load.
  constexpr double capacityAh = season::simulation::runtimeconfig::kBatteryCapacityAh;
  struct BatteryState
  {
    double remainingAh = capacityAh;
    double previousRuntimeTime = -1.0;
    int brownoutCount = 0;
    bool previouslyBrownedOut = false;
  };
  static BatteryState battery;
  const double now = runtime.Time();
  const double dt = battery.previousRuntimeTime >= 0 && now >= battery.previousRuntimeTime
                        ? now - battery.previousRuntimeTime
                        : 0.0;
  battery.previousRuntimeTime = now;
  const double current = runtime.SupplyCurrent();
  battery.remainingAh = std::max(0.0, battery.remainingAh - current * dt / 3600.0);
  const double stateOfCharge = std::clamp(battery.remainingAh / capacityAh, 0.0, 1.0);
  const double openCircuitVoltage = season::simulation::runtimeconfig::kBatteryEmptyVoltage + season::simulation::runtimeconfig::kBatteryVoltageRange * std::sqrt(stateOfCharge);
  const double batteryVoltage = meco::sim::UpdateBatterySupply(
      runtime, meco::sim::BatterySupplyConfig{openCircuitVoltage, season::simulation::runtimeconfig::kBatteryInternalResistanceOhms, 0.0});
  const bool brownedOut = frc::RobotController::IsBrownedOut();
  if (brownedOut && !battery.previouslyBrownedOut)
    ++battery.brownoutCount;
  battery.previouslyBrownedOut = brownedOut;
  frc::SmartDashboard::PutNumber("SimulationPower/SupplyCurrentAmps", runtime.SupplyCurrent());
  frc::SmartDashboard::PutNumber("SimulationPower/BusVoltage", batteryVoltage);
  frc::SmartDashboard::PutNumber("SimulationPower/BatteryRemainingAh", battery.remainingAh);
  frc::SmartDashboard::PutNumber("SimulationPower/BatteryStateOfCharge", stateOfCharge);
  frc::SmartDashboard::PutBoolean("SimulationPower/BrownedOut", brownedOut);
  frc::SmartDashboard::PutNumber("SimulationPower/BrownoutCount", battery.brownoutCount);
  static auto powerPublisher =
      nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/PowerSamples").Publish();
  powerPublisher.Set(std::array<double, 3>{static_cast<double>(nt::Now()), runtime.SupplyCurrent(), batteryVoltage});
  static auto batteryPublisher =
      nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/BatteryState").Publish();
  batteryPublisher.Set(std::array<double, 4>{battery.remainingAh, stateOfCharge,
                                             brownedOut ? 1.0 : 0.0,
                                             static_cast<double>(battery.brownoutCount)});
  meco::sim::PublishDriverStationPacket();
#endif
}
void Robot::DisabledInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
}
