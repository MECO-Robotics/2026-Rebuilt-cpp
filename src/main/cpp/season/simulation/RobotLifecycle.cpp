#include "season/detail/RobotLifecycle.h"
#include "base/SimulationLatency.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include "meco/sim/SimulationLifecycle.h"
#endif

namespace season::runtime
{
  struct RobotLifecycle::State
  {
#ifndef __FRC_ROBORIO__
    meco::sim::SimulationLifecycle lifecycle{meco::sim::SimulationLifecycleConfig{
        season::simulation::runtimeconfig::kBatteryCapacityAh,
        season::simulation::runtimeconfig::kBatteryEmptyVoltage,
        season::simulation::runtimeconfig::kBatteryVoltageRange,
        season::simulation::runtimeconfig::kBatteryInternalResistanceOhms,
        season::simulation::runtimeconfig::kPracticeAutonomousSeconds,
        season::simulation::runtimeconfig::kPracticeAssessmentSeconds,
        season::simulation::runtimeconfig::kPracticeTeleopSeconds}};
#endif
  };

  RobotLifecycle::RobotLifecycle() : state_(std::make_unique<State>()) {}
  RobotLifecycle::~RobotLifecycle() = default;
  void RobotLifecycle::Initialize()
  {
#ifndef __FRC_ROBORIO__
    state_->lifecycle.Initialize();
#endif
  }
  void RobotLifecycle::BeforeScheduler()
  {
#ifndef __FRC_ROBORIO__
    state_->lifecycle.BeforeScheduler();
#else
    base::simulation::BeginLatencyProbe();
#endif
  }
  void RobotLifecycle::AfterScheduler()
  {
#ifndef __FRC_ROBORIO__
    state_->lifecycle.AfterScheduler();
#else
    base::simulation::EndLatencyProbe();
#endif
  }
  void RobotLifecycle::BeforeScene()
  {
#ifndef __FRC_ROBORIO__
    state_->lifecycle.BeforeScene();
#endif
  }
  void RobotLifecycle::AfterScene()
  {
#ifndef __FRC_ROBORIO__
    state_->lifecycle.AfterScene();
#endif
  }
}
