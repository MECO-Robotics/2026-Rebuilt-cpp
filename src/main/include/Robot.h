#pragma once
#include <frc/TimedRobot.h>
#include "season/detail/RobotLifecycle.h"
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
  season::runtime::RobotLifecycle lifecycle_;
  frc2::Command *autonomous_ = nullptr;
  bool autonomousPending_ = false;
};
