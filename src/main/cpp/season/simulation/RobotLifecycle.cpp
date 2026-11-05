#include "Robot.h"
#include "season/simulation/SimulationConfig.h"
#include "base/SimulationLifecycle.h"

namespace season::runtime
{
  struct RobotLifecycle::State
  {
    base::simulation::Lifecycle lifecycle{base::simulation::LifecycleConfig{
        season::simulation::runtimeconfig::kBatteryCapacityAh,
        season::simulation::runtimeconfig::kBatteryEmptyVoltage,
        season::simulation::runtimeconfig::kBatteryVoltageRange,
        season::simulation::runtimeconfig::kBatteryInternalResistanceOhms,
        season::simulation::runtimeconfig::kPracticeAutonomousSeconds,
        season::simulation::runtimeconfig::kPracticeAssessmentSeconds,
        season::simulation::runtimeconfig::kPracticeTeleopSeconds}};
  };

  RobotLifecycle::RobotLifecycle() : state_(std::make_unique<State>()) {}
  RobotLifecycle::~RobotLifecycle() = default;
  void RobotLifecycle::Initialize() { state_->lifecycle.Initialize(); }
  void RobotLifecycle::BeforeScheduler() { state_->lifecycle.BeforeScheduler(); }
  void RobotLifecycle::AfterScheduler() { state_->lifecycle.AfterScheduler(); }
  void RobotLifecycle::BeforeScene() { state_->lifecycle.BeforeScene(); }
  void RobotLifecycle::AfterScene() { state_->lifecycle.AfterScene(); }
}
