#pragma once
#include <frc/TimedRobot.h>
#include "RobotContainer.h"

// Drives the command scheduler and season mode transitions.
class Robot : public frc::TimedRobot
{
public:
  Robot();
  ~Robot() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;
  void SimulationPeriodic() override;
  void DisabledInit() override;
  // Expose WPILib's normal mode-transition loop to MecoBase's offline replay runner.
  void ReplayLoop() { LoopFunc(); }

private:
  RobotContainer container_;
  frc2::Command *autonomous_ = nullptr;
  bool autonomousPending_ = false;
  void RunRobotPeriodic();
};
