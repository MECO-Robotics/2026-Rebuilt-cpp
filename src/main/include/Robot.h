#pragma once
#include <frc/TimedRobot.h>
#include <networktables/DoubleArrayTopic.h>
#include <cstdint>
#include "RobotContainer.h"
#ifndef __FRC_ROBORIO__
#include <meco/sim/PracticeMatchController.h>
#endif
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
  void FastControlPeriodic();
  RobotContainer container_;
#ifndef __FRC_ROBORIO__
  std::unique_ptr<meco::sim::PracticeMatchController> practice_;
#endif
  frc2::Command *autonomous_ = nullptr;
  bool autonomousPending_ = false;
  bool fastControlEnabled_ = false;
  int64_t nextFastControlFpgaUs_{};
  uint64_t fastControlSequence_{};
  nt::DoubleArrayPublisher fastControlSamplesPublisher_;
};
