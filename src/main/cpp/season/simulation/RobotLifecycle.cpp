#include "season/detail/RobotLifecycle.h"
#include "base/SimulationLatency.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include "base/SimulationRuntime.h"
#include "meco/sim/BatterySupply.h"
#include <meco/sim/PracticeMatchController.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#endif

namespace season::runtime
{
  struct RobotLifecycle::State
  {
#ifndef __FRC_ROBORIO__
    std::unique_ptr<meco::sim::PracticeMatchController> practice;
#endif
  };

  RobotLifecycle::RobotLifecycle() : state_(std::make_unique<State>()) {}
  RobotLifecycle::~RobotLifecycle() = default;

  void RobotLifecycle::Initialize()
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
    if (frc::RobotBase::IsSimulation())
      state_->practice = std::make_unique<meco::sim::PracticeMatchController>(
          meco::sim::PracticeMatchProfile{season::simulation::runtimeconfig::kPracticeAutonomousSeconds,
                                         season::simulation::runtimeconfig::kPracticeAssessmentSeconds,
                                         season::simulation::runtimeconfig::kPracticeTeleopSeconds});
#endif
  }

  void RobotLifecycle::BeforeScheduler() { base::simulation::BeginLatencyProbe(); }
  void RobotLifecycle::AfterScheduler() { base::simulation::EndLatencyProbe(); }

  void RobotLifecycle::BeforeScene()
  {
#ifndef __FRC_ROBORIO__
    if (state_->practice)
      state_->practice->Update();
#endif
  }

  void RobotLifecycle::AfterScene()
  {
#ifndef __FRC_ROBORIO__
    auto &runtime = base::simulation::GetSimulationRuntime();
    static auto physicsClockPublisher = nt::NetworkTableInstance::GetDefault()
                                            .GetDoubleArrayTopic("/Simulation/Physics/Samples")
                                            .Publish();
    physicsClockPublisher.Set(std::array<double, 3>{static_cast<double>(nt::Now()),
                                                    static_cast<double>(frc::RobotController::GetFPGATime()), runtime.Time()});
    // Battery state lasts for the simulation process, including scene resets.
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
    const double openCircuitVoltage = season::simulation::runtimeconfig::kBatteryEmptyVoltage +
                                      season::simulation::runtimeconfig::kBatteryVoltageRange * std::sqrt(stateOfCharge);
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
}
