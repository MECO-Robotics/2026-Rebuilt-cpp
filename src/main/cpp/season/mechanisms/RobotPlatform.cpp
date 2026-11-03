#include "season/mechanisms/RobotPlatform.h"
#include "season/drive/FastControlProfile.h"
#include <frc/RobotBase.h>
#include <cstdlib>
#include <string_view>
#include <ctre/phoenix6/CANBus.hpp>
namespace season::hardware {
  // Select hardware IO only on a real robot.
  base::Mode Mode() { return frc::RobotBase::IsReal() ? base::Mode::Real : base::Mode::Sim; }
  bool FastControlEnabled()
  {
    const char *value = std::getenv("FRC_FAST_CONTROL");
    return value ? std::string_view{value} == "1" : season::drive::fastcontrol::Config().enabled;
  }
  base::MotorConfig MotorRates(base::MotorConfig config, base::Vendor vendor)
  {
    const auto &profile = season::drive::fastcontrol::Config();
    if (vendor == base::Vendor::SparkMax)
    {
      config.rates.positionVelocityHz = profile.sparkStatusHz;
      config.rates.controlHz = 1000.0 / profile.sparkControlMs;
    }
    else
    {
      const bool canFd = !frc::RobotBase::IsReal() ||
                         (!config.canBus.empty() && ctre::phoenix6::CANBus{config.canBus}.IsNetworkFD());
      config.rates.positionVelocityHz = canFd ? profile.talonFdStatusHz : profile.talonClassicStatusHz;
      config.rates.controlHz = canFd ? 1000.0 / profile.talonFdControlMs : 1000.0 / profile.talonClassicControlMs;
    }
    return config;
  }
  base::Gains JointGains(base::Gains gains) { return frc::RobotBase::IsSimulation() ? SimulationGainsFor(gains) : gains; }
  bool MechanismCanFd() {
    return !frc::RobotBase::IsReal() || (!kBottomIndexer.motor.canBus.empty() && ctre::phoenix6::CANBus{kBottomIndexer.motor.canBus}.IsNetworkFD());
  }
  const char *SystemCheckPath() { return frc::RobotBase::IsReal() ? "/home/lvuser/systemcheck-latest.json" : "systemcheck-latest.json"; }
}
