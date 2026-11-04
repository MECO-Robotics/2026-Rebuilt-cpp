#include "season/detail/RobotPlatform.h"
#include "season/mechanisms/Mechanisms.h"
#include <frc/RobotBase.h>
#include <ctre/phoenix6/CANBus.hpp>
namespace season::hardware {
  // Select hardware IO only on a real robot.
  base::Mode Mode() { return frc::RobotBase::IsReal() ? base::Mode::Real : base::Mode::Sim; }
  base::MotorConfig MotorRates(base::MotorConfig config, base::Vendor vendor)
  {
    if (vendor == base::Vendor::SparkMax)
    {
      config.rates.positionVelocityHz = kSparkStatusHz;
      config.rates.controlHz = kSparkControlHz;
    }
    else
    {
      const bool canFd = !frc::RobotBase::IsReal() ||
                         (!config.canBus.empty() && ctre::phoenix6::CANBus{config.canBus}.IsNetworkFD());
      config.rates.positionVelocityHz = canFd ? kTalonFdStatusHz : kTalonClassicStatusHz;
      config.rates.controlHz = canFd ? kTalonFdControlHz : kTalonClassicControlHz;
    }
    return config;
  }
  base::Gains JointGains(base::Gains gains) { return frc::RobotBase::IsSimulation() ? SimulationGainsFor(gains) : gains; }
  const char *SystemCheckPath() { return frc::RobotBase::IsReal() ? "/home/lvuser/systemcheck-latest.json" : "systemcheck-latest.json"; }
}
