#pragma once

#include <memory>

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
