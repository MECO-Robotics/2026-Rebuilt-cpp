#pragma once
#include <frc/TimedRobot.h>
#include "RobotContainer.h"
#ifndef __FRC_ROBORIO__
#include <meco/sim/PracticeMatchController.h>
#endif
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
#ifndef __FRC_ROBORIO__
  std::unique_ptr<meco::sim::PracticeMatchController> practice_;
#endif
  frc2::Command *autonomous_ = nullptr;
  bool autonomousPending_ = false;
};
