#pragma once
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/mechanisms/Mechanisms.h"
#include "base/NtGamepad.h"
#include "base/DriverInput.h"
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include <frc2/command/CommandPtr.h>
class RobotContainer;
namespace season { class AutoDriveEnvironment; }
namespace season::runtime {
class RobotRuntime {
public:
  explicit RobotRuntime(RobotContainer &owner);
  ~RobotRuntime();
  base::MotorIOSelector &Motors();
  base::DriveConfig ConfigureDrive(base::DriveConfig config);
  std::vector<std::unique_ptr<base::VisionIO>> LocalizationSources();
  void Initialize();
  void InitializeFeedback();
  void BeforeScheduler();
  void AfterScheduler();
  void BeforeScene();
  void AfterScene();
  std::shared_ptr<season::AutoDriveEnvironment> AutoDriveInputs();
  void FinishSetup();
  bool UseNtInput(bool selected) const;
  std::optional<frc2::CommandPtr> PracticeMatchCommand();
  const std::function<void(const base::DriverInputState &, double, double, double, frc::Rotation2d)> *DriveRequestObserver() const;
  const std::vector<int> *TagIds() const;
  void BindReset();
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
