#ifndef __FRC_ROBORIO__
#include "season/simulation/LoadedMotorSimulation.h"
#include "season/simulation/MotorModelConfig.h"
#include "base/SimulationRuntime.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace season
{
  namespace
  {
    class LoadedMotorIO final : public base::MotorIO
    {
    public:
      LoadedMotorIO(base::MotorConfig config, base::Vendor vendor, double inertia,
                    std::shared_ptr<FuelMotorLoads> loads, LoadedMotor motor)
          : config_(std::move(config)), inertia_(inertia), loads_(std::move(loads)), motor_(motor),
            resistance_(vendor == base::Vendor::SparkMax ? simulation::motorconfig::kNominalVoltage / simulation::motorconfig::kNeoStallCurrentAmps : simulation::motorconfig::kNominalVoltage / simulation::motorconfig::kKrakenStallCurrentAmps),
            kt_(vendor == base::Vendor::SparkMax ? simulation::motorconfig::kNeoStallTorqueNm / simulation::motorconfig::kNeoStallCurrentAmps : simulation::motorconfig::kKrakenStallTorqueNm / simulation::motorconfig::kKrakenStallCurrentAmps),
            kv_(vendor == base::Vendor::SparkMax ? simulation::motorconfig::kNeoFreeSpeedRpm * 2 * std::numbers::pi / 60 / simulation::motorconfig::kNominalVoltage
                                                 : simulation::motorconfig::kKrakenFreeSpeedRpm * 2 * std::numbers::pi / 60 / simulation::motorconfig::kNominalVoltage)
      {
        config_.Validate();
        if (!(inertia_ > 0))
          throw std::invalid_argument("invalid motor inertia");
        registration_ = base::simulation::GetSimulationRuntime().Register({[this](double dt)
                                                                           { bus_ = base::simulation::GetSimulationRuntime().BusVoltage(); StepSimulation(units::second_t{dt}); },
                                                                           {},
                                                                           {},
                                                                           [this]
                                                                           { position_ = velocity_ = voltage_ = request_ = current_ = 0; bus_ = simulation::motorconfig::kNominalVoltage; },
                                                                           [this]
                                                                           { return SupplyCurrent(); }});
      }
      base::MotorInputs Read() override
      {
        return {position_ / (2 * std::numbers::pi), velocity_ / (2 * std::numbers::pi), voltage_, bus_, SupplyCurrent(), true};
      }
      void SetVoltage(double volts) override
      {
        if (!std::isfinite(volts))
          throw std::invalid_argument("invalid motor voltage");
        request_ = volts;
      }
      void SetBrake(bool brake) override { brake_ = brake; }
      void StepSimulation(units::second_t duration) override
      {
        const double dt = duration.value();
        if (!std::isfinite(dt) || dt < 0 || dt > simulation::motorconfig::kMaxUpdateSeconds)
          throw std::invalid_argument("invalid simulation step");
        const int steps = std::max(1, static_cast<int>(std::ceil(dt / simulation::motorconfig::kMaxPhysicsStepSeconds)));
        const double h = dt / steps;
        // These inputs cannot change during a single simulation step.
        const double requestedVoltage = std::clamp(request_, -bus_, bus_);
        const double load = std::max(0., loads_->torque[static_cast<size_t>(motor_)]);
        const double torquePerAmp = kt_ * config_.ids.size() * config_.gearRatio;
        const double currentLimit = static_cast<double>(config_.currentLimit);
        for (int i = 0; i < steps; ++i)
        {
          voltage_ = requestedVoltage;
          current_ = (!brake_ && std::abs(voltage_) < 1e-9) ? 0 : std::clamp((voltage_ - config_.gearRatio * velocity_ / kv_) / resistance_, -currentLimit, currentLimit);
          const double opposing = std::abs(velocity_) > simulation::motorconfig::kLoadDirectionSpeedThreshold ? std::copysign(load, velocity_) : std::clamp(torquePerAmp * current_, -load, load);
          velocity_ += (torquePerAmp * current_ - opposing - simulation::motorconfig::kViscousDrag * velocity_) / inertia_ * h;
          position_ += velocity_ * h;
          voltage_ = std::clamp(resistance_ * current_ + config_.gearRatio * velocity_ / kv_, -std::abs(voltage_), std::abs(voltage_));
        }
      }

    private:
      double SupplyCurrent() const { return bus_ > 0 ? std::max(0., voltage_ * current_) * config_.ids.size() / bus_ : 0; }
      base::MotorConfig config_;
      double inertia_;
      std::shared_ptr<FuelMotorLoads> loads_;
      LoadedMotor motor_;
      double resistance_, kt_, kv_, position_{}, velocity_{}, voltage_{}, request_{}, current_{}, bus_{simulation::motorconfig::kNominalVoltage};
      bool brake_{true};
      base::simulation::Registration registration_;
    };
  }
  std::unique_ptr<base::MotorIO> MakeLoadedMotorIO(const base::MotorConfig &config, base::Vendor vendor, double inertia,
                                                   std::shared_ptr<FuelMotorLoads> loads, LoadedMotor motor)
  {
    return std::make_unique<LoadedMotorIO>(config, vendor, inertia, std::move(loads), motor);
  }
}
#endif
