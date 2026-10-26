#include "Robot.h"
#include <string_view>
#include "season/SimulationLatency.h"
#include "season/HubShift.h"
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
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
void Robot::RobotPeriodic()
{
  season::simulation::BeginLatencyProbe();
  frc2::CommandScheduler::GetInstance().Run();
  season::simulation::EndLatencyProbe();
  auto alliance = frc::DriverStation::GetAlliance();
  std::optional<season::Alliance> own;
  if (alliance)
    own = *alliance == frc::DriverStation::Alliance::kBlue ? season::Alliance::Blue : season::Alliance::Red;
  auto info = season::ComputeHubShift(frc::DriverStation::GetMatchTime().value(), frc::DriverStation::IsTeleop(), own, frc::DriverStation::GetGameSpecificMessage());
  int seconds = static_cast<int>(info.matchTime);
  frc::SmartDashboard::PutString("Match Time", fmt::format("{:02d}:{:02d}.{}", seconds / 60, seconds % 60, static_cast<int>(info.matchTime * 10) % 10));
  frc::SmartDashboard::PutString("Shifts/Remaining Shift Time", fmt::format("{:.1f}", info.remainingTime));
  frc::SmartDashboard::PutBoolean("Shifts/Shift Active", info.active);
  frc::SmartDashboard::PutString("Shifts/Game State", frc::DriverStation::IsAutonomous() ? "Autonomous" : season::ShiftStateName(info.currentShift));
  frc::SmartDashboard::PutBoolean("Shifts/Active First?", own.value_or(season::Alliance::Blue) == season::FirstActiveAlliance(frc::DriverStation::GetGameSpecificMessage()));
  frc::SmartDashboard::PutString("Shifts/Match Time Color", info.matchTimeColor);
  frc::SmartDashboard::PutString("Shifts/Shift Time Color", info.shiftTimeColor);
}
void Robot::AutonomousInit()
{
  container_.CancelChecks();
  container_.ResetSimulation();
  autonomous_ = container_.GetAutonomousCommand();
  if (autonomous_)
    autonomous_->Schedule();
}
void Robot::TeleopInit()
{
  container_.CancelChecks();
  if (autonomous_)
    autonomous_->Cancel();
}
void Robot::TestInit()
{
  container_.CancelChecks();
  frc2::CommandScheduler::GetInstance().CancelAll();
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

void Robot::SimulationPeriodic()
{
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
void Robot::DisabledInit() { container_.CancelChecks(); }
