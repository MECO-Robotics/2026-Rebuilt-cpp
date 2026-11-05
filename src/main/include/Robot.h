#pragma once
#include <frc/TimedRobot.h>
#include <memory>
#include "RobotContainer.h"
namespace season::runtime
{
  // Platform services surrounding the robot scheduler and scene updates.
  class RobotLifecycle
  {
  public:
    RobotLifecycle();
    ~RobotLifecycle();
    RobotLifecycle(const RobotLifecycle &) = delete;
    RobotLifecycle &operator=(const RobotLifecycle &) = delete;
    void Initialize();
    void BeforeScheduler();
    void AfterScheduler();
    void BeforeScene();
    void AfterScene();

  private:
    struct State;
    std::unique_ptr<State> state_;
  };
}

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
