#ifndef __FRC_ROBORIO__
#include "season/simulation/LoadedMotorSimulation.h"
#include "base/SimulationRuntime.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace season {
namespace {
class LoadedMotorIO final : public base::MotorIO {
 public:
  LoadedMotorIO(base::MotorConfig config, base::Vendor vendor, double inertia,
                std::shared_ptr<FuelMotorLoads> loads, LoadedMotor motor)
      : config_(std::move(config)), inertia_(inertia), loads_(std::move(loads)), motor_(motor),
        resistance_(vendor == base::Vendor::SparkMax ? 12. / 105 : 12. / 366),
        kt_(vendor == base::Vendor::SparkMax ? 2.6 / 105 : 7.09 / 366),
        kv_(vendor == base::Vendor::SparkMax ? 5676 * 2 * std::numbers::pi / 60 / 12
                                             : 6000 * 2 * std::numbers::pi / 60 / 12) {
    config_.Validate();
    if (!(inertia_ > 0)) throw std::invalid_argument("invalid motor inertia");
    registration_ = base::simulation::GetSimulationRuntime().Register({
      [this](double dt) { bus_ = base::simulation::GetSimulationRuntime().BusVoltage(); StepSimulation(units::second_t{dt}); },
      {}, {}, [this] { position_ = velocity_ = voltage_ = request_ = current_ = 0; bus_ = 12; },
      [this] { return SupplyCurrent(); }});
  }
  base::MotorInputs Read() override {
    return {position_ / (2 * std::numbers::pi), velocity_ / (2 * std::numbers::pi), voltage_, bus_, SupplyCurrent(), true};
  }
  void SetVoltage(double volts) override {
    if (!std::isfinite(volts)) throw std::invalid_argument("invalid motor voltage");
    request_ = volts;
  }
  void SetBrake(bool brake) override { brake_ = brake; }
  void StepSimulation(units::second_t duration) override {
    const double dt = duration.value();
    if (!std::isfinite(dt) || dt < 0 || dt > 1) throw std::invalid_argument("invalid simulation step");
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / .0002)));
    const double h = dt / steps;
    for (int i = 0; i < steps; ++i) {
      voltage_ = std::clamp(request_, -bus_, bus_);
      current_ = (!brake_ && std::abs(voltage_) < 1e-9) ? 0 :
          std::clamp((voltage_ - config_.gearRatio * velocity_ / kv_) / resistance_,
                     -static_cast<double>(config_.currentLimit), static_cast<double>(config_.currentLimit));
      const double load = std::max(0., loads_->torque[static_cast<size_t>(motor_)]);
      const double opposing = std::abs(velocity_) > .01 ? std::copysign(load, velocity_) :
                              std::clamp(kt_ * config_.ids.size() * config_.gearRatio * current_, -load, load);
      velocity_ += (kt_ * config_.ids.size() * config_.gearRatio * current_ - opposing - .0001 * velocity_) / inertia_ * h;
      position_ += velocity_ * h;
      voltage_ = std::clamp(resistance_ * current_ + config_.gearRatio * velocity_ / kv_, -std::abs(voltage_), std::abs(voltage_));
    }
  }
 private:
  double SupplyCurrent() const { return bus_ > 0 ? std::max(0., voltage_ * current_) * config_.ids.size() / bus_ : 0; }
  base::MotorConfig config_; double inertia_; std::shared_ptr<FuelMotorLoads> loads_; LoadedMotor motor_;
  double resistance_, kt_, kv_, position_{}, velocity_{}, voltage_{}, request_{}, current_{}, bus_{12};
  bool brake_{true};
  base::simulation::Registration registration_;
};
}
std::unique_ptr<base::MotorIO> MakeLoadedMotorIO(const base::MotorConfig& config, base::Vendor vendor, double inertia,
                                                std::shared_ptr<FuelMotorLoads> loads, LoadedMotor motor) {
  return std::make_unique<LoadedMotorIO>(config, vendor, inertia, std::move(loads), motor);
}
}
#endif
