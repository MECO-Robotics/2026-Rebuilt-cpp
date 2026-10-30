#include "Robot.h"
#include <string_view>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include "base/SimulationLatency.h"
#include "season/MatchContext.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/format.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#ifndef __FRC_ROBORIO__
#include "base/SimulationRuntime.h"
#include "meco/sim/BatterySupply.h"
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <charconv>
#include <stdexcept>
#endif
#include <frc2/command/CommandScheduler.h>
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
    practice_ = std::make_unique<meco::sim::PracticeMatchController>(meco::sim::PracticeMatchProfile{20,3,140});
#endif
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
void Robot::RobotPeriodic()
{
  const auto match = season::ReadMatchContext();
  container_.SetMatchContext(match);
  if (autonomousPending_ && frc::DriverStation::IsAutonomousEnabled() && container_.SimulationResetReady()) {
    autonomousPending_ = false;
    if (autonomous_) autonomous_->Schedule();
  }
  base::simulation::BeginLatencyProbe();
  frc2::CommandScheduler::GetInstance().Run();
  base::simulation::EndLatencyProbe();
  auto nt = nt::NetworkTableInstance::GetDefault().GetTable("/Match");
  nt->GetEntry("Phase").SetString(match.phase);
  nt->GetEntry("TimeKnown").SetBoolean(match.periodTime.has_value());
  nt->GetEntry("Time").SetDouble(match.periodTime.value_or(-1));
  nt->GetEntry("GameDataValid").SetBoolean(match.gameDataValid);
  nt->GetEntry("GameData").SetString(match.gameData);
  nt->GetEntry("InactiveFirstAlliance").SetString(season::AllianceLabel(match.inactiveFirstAlliance));
  nt->GetEntry("FirstActiveAlliance").SetString(season::AllianceLabel(match.firstActiveAlliance));
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
  frc::SmartDashboard::PutString("Shifts/Game State",match.phase);
  frc::SmartDashboard::PutBoolean("Shifts/Shift Active",match.ownActive.value_or(false));
  frc::SmartDashboard::PutBoolean("Shifts/State Known",match.ownActive.has_value());
  if (match.periodTime) {
    const int seconds=static_cast<int>(*match.periodTime);
    frc::SmartDashboard::PutString("Match Time",fmt::format("{:02d}:{:02d}.{}",seconds/60,seconds%60,static_cast<int>(*match.periodTime*10)%10));
  } else frc::SmartDashboard::PutString("Match Time","unknown");
  const auto next=match.alliance ? (*match.alliance==season::Alliance::Blue?match.blueNextChange:match.redNextChange) : std::nullopt;
  frc::SmartDashboard::PutString("Shifts/Remaining Shift Time",next?fmt::format("{:.1f}",*next):"unknown");
  frc::SmartDashboard::PutBoolean("Shifts/Active First?",match.alliance&&match.gameDataValid&&
      ((*match.alliance==season::Alliance::Blue)==(match.gameData=="R")));

}
void Robot::AutonomousInit()
{
  container_.CancelChecks();
  // Scheduling initializes AutoDrive immediately, before RobotPeriodic runs.
  // Supply this packet's context for initialization as well as execution.
  container_.SetMatchContext(season::ReadMatchContext());
  container_.ResetSimulation();
  autonomous_ = container_.GetAutonomousCommand();
  autonomousPending_ = autonomous_ != nullptr;
  if (autonomousPending_ && container_.SimulationResetReady()) {
    autonomousPending_ = false;
    autonomous_->Schedule();
  }
}
void Robot::TeleopInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
  if (autonomous_)
    autonomous_->Cancel();
}
void Robot::TestInit()
{
  autonomousPending_ = false;
  container_.CancelChecks();
  frc2::CommandScheduler::GetInstance().CancelAll();
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

void Robot::SimulationPeriodic()
{
#ifndef __FRC_ROBORIO__
  if (practice_) practice_->Update();
#endif
  container_.SimulationPeriodic();
#ifndef __FRC_ROBORIO__
  auto &runtime = base::simulation::GetSimulationRuntime();
  const double batteryVoltage = meco::sim::UpdateBatterySupply(runtime);
  frc::SmartDashboard::PutNumber("SimulationPower/SupplyCurrentAmps", runtime.SupplyCurrent());
  frc::SmartDashboard::PutNumber("SimulationPower/BusVoltage", batteryVoltage);
  static auto powerPublisher = nt::NetworkTableInstance::GetDefault()
                                   .GetDoubleArrayTopic("/Simulation/PowerSamples")
                                   .Publish();
  powerPublisher.Set(std::array<double, 3>{static_cast<double>(nt::Now()), runtime.SupplyCurrent(), batteryVoltage});
  meco::sim::PublishDriverStationPacket();
#endif
}
void Robot::DisabledInit() { autonomousPending_ = false; container_.CancelChecks(); }
