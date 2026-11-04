#include "season/detail/RobotPlatform.h"
#include "season/drive/DriveConfig.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/DriveSimulationConfig.h"
#include "season/simulation/MechanismSimulationConfig.h"
#endif
#include "season/mechanisms/Mechanisms.h"
#include <frc/RobotBase.h>
#include <ctre/phoenix6/CANBus.hpp>
#include <utility>
namespace season::hardware {
  base::DriveConfig DriveConfig()
  {
    auto config = HardwareDriveConfig(kSwerveOdometryFdHz);
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation())
      return season::simulation::ConfigureDrive(std::move(config));
#endif
    config.odometryHz = ctre::phoenix6::CANBus{config.bus}.IsNetworkFD()
                            ? kSwerveOdometryFdHz : kSwerveOdometryClassicHz;
    return config;
  }
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
  base::Gains JointGains(base::Gains gains)
  {
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation())
      return simulation::ConfigureJointGains(gains);
#endif
    return gains;
  }
  const char *SystemCheckPath() { return frc::RobotBase::IsReal() ? "/home/lvuser/systemcheck-latest.json" : "systemcheck-latest.json"; }
}
