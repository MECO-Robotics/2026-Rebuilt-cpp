#pragma once
#include "season/runtime/RobotRuntime.h"
#include "season/simulation/MotorSimulation.h"
#include <base/RuntimeScene.h>
#include <base/VisionIOSelector.h>
#include <utility>

namespace season::runtime {
struct RobotRuntime::Simulation : base::RuntimeScene {
  using VisionProviders = std::pair<base::SimVisionProvider, base::SimVisionProvider>;
  virtual VisionProviders LocalizationProviders() { return {}; }
  virtual std::shared_ptr<season::AutoDriveEnvironment> AutoDriveInputs() { return {}; }
  virtual bool UseNtInput(bool selected) const { return selected; }
  virtual base::DriveConfig ConfigureDrive(base::DriveConfig config) { return config; }
  virtual std::optional<frc2::CommandPtr> PracticeMatchCommand() { return std::nullopt; }
  virtual const std::function<void(const base::DriverInputState &, double, double, double, frc::Rotation2d)> *DriveRequestObserver() const { return nullptr; }
  virtual const std::vector<int> *TagIds() const { return nullptr; }
  virtual void BeforeScheduler() {}
  virtual void AfterScheduler() {}
  virtual void BeforeScene() {}
  virtual void AfterScene() {}
};
struct RobotRuntime::State {
  std::shared_ptr<season::FuelMotorLoads> fuelMotorLoads_ = std::make_shared<season::FuelMotorLoads>();
  base::MotorIOSelector motorSelector_{season::simulation::MotorProvider(fuelMotorLoads_)};
  std::unique_ptr<Simulation> simulation_;
};
}
