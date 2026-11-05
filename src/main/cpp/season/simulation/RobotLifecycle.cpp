#include "season/detail/RobotLifecycle.h"
#include "base/SimulationLatency.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include "base/SimulationRuntime.h"
#include "meco/sim/SimulationPower.h"
#include "meco/sim/SimulationNetwork.h"
#include <meco/sim/PracticeMatchController.h>
#include <frc/RobotBase.h>
#endif

namespace season::runtime
{
  struct RobotLifecycle::State
  {
#ifndef __FRC_ROBORIO__
    std::unique_ptr<meco::sim::PracticeMatchController> practice;
    std::unique_ptr<meco::sim::SimulationPower> power;
#endif
  };

  RobotLifecycle::RobotLifecycle() : state_(std::make_unique<State>()) {}
  RobotLifecycle::~RobotLifecycle() = default;

  void RobotLifecycle::Initialize()
  {
#ifndef __FRC_ROBORIO__
    meco::sim::ConfigureSimulationNetwork();
    state_->power = std::make_unique<meco::sim::SimulationPower>(meco::sim::SimulationPowerConfig{
        season::simulation::runtimeconfig::kBatteryCapacityAh,
        season::simulation::runtimeconfig::kBatteryEmptyVoltage,
        season::simulation::runtimeconfig::kBatteryVoltageRange,
        season::simulation::runtimeconfig::kBatteryInternalResistanceOhms});
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
    state_->power->Update(runtime);
    meco::sim::PublishDriverStationPacket();
#endif
  }
}
