#pragma once
#include "season/detail/RobotRuntime.h"
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
  virtual void PublishDriverInput(const base::NtGamepad::State &, double, double, double, double, double, double) {}
  virtual const std::vector<int> *TagIds() const { return nullptr; }
};
struct RobotRuntime::State {
  std::shared_ptr<season::FuelMotorLoads> fuelMotorLoads_ = std::make_shared<season::FuelMotorLoads>();
  base::MotorIOSelector motorSelector_{season::simulation::MotorProvider(fuelMotorLoads_)};
  std::unique_ptr<Simulation> simulation_;
};
}
