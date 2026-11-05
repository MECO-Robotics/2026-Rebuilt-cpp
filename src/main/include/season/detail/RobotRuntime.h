#pragma once
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/mechanisms/Mechanisms.h"
#include "base/NtGamepad.h"
#include "season/drive/AutoDriveEnvironment.h"
#include <memory>
#include <vector>
class RobotContainer;
namespace season::runtime {
class RobotRuntime {
public:
  explicit RobotRuntime(RobotContainer &owner);
  ~RobotRuntime();
  base::MotorIOSelector &Motors();
  std::vector<std::unique_ptr<base::VisionIO>> LocalizationSources();
  void Initialize();
  void InitializeFeedback();
  std::shared_ptr<season::AutoDriveEnvironment> AutoDriveInputs();
  void FinishSetup();
  bool UseNtInput(bool selected) const;
  void PublishDriverInput(const base::NtGamepad::State &, double, double, double, double, double, double);
  const std::vector<int> *TagIds() const;
  void BindReset();
  void Schedule(frc2::Command *);
  void Periodic();
  void Reset();
  bool ResetReady() const;
private:
  RobotContainer &owner_;
  struct Simulation;
  struct DesktopSimulation;
  std::unique_ptr<Simulation> MakeDesktopSimulation();
  struct State;
  std::unique_ptr<State> state_;
 };
}
