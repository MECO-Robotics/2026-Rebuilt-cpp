#pragma once
#include "season/simulation/SimulationConfig.h"
#include <base/SimulationLifecycle.h>
#include <frc/TimedRobot.h>
#include "RobotContainer.h"

// Drives the command scheduler and season mode transitions.
class Robot : public frc::TimedRobot
{
public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;
  void SimulationPeriodic() override;
  void DisabledInit() override;

private:
  RobotContainer container_;
  base::simulation::Lifecycle lifecycle_{base::simulation::LifecycleConfig{
      season::simulation::runtimeconfig::kBatteryCapacityAh,
      season::simulation::runtimeconfig::kBatteryEmptyVoltage,
      season::simulation::runtimeconfig::kBatteryVoltageRange,
      season::simulation::runtimeconfig::kBatteryInternalResistanceOhms,
      season::simulation::runtimeconfig::kPracticeAutonomousSeconds,
      season::simulation::runtimeconfig::kPracticeAssessmentSeconds,
      season::simulation::runtimeconfig::kPracticeTeleopSeconds}};
  frc2::Command *autonomous_ = nullptr;
  bool autonomousPending_ = false;
};
