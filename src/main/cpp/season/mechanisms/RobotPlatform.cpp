#include "season/detail/RobotPlatform.h"
#include "season/drive/DriveConfig.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/DriveSimulationConfig.h"
#include "season/simulation/MechanismSimulationConfig.h"
#endif
#include "season/mechanisms/Mechanisms.h"
#include <base/CanBusMode.h>
#include <base/RuntimeMode.h>
#include <utility>
namespace season::hardware {
  base::DriveConfig DriveConfig()
  {
    auto config = HardwareDriveConfig(kSwerveOdometryFdHz);
#ifndef __FRC_ROBORIO__
    if (base::CurrentRuntimeMode() == base::Mode::Sim)
      return season::simulation::ConfigureDrive(std::move(config));
#endif
    config.odometryHz = base::SelectCanBusRate(config.bus, kSwerveOdometryFdHz, kSwerveOdometryClassicHz);
    return config;
  }
  base::MotorConfig MotorRates(base::MotorConfig config, base::Vendor vendor)
  {
    if (vendor == base::Vendor::SparkMax)
    {
      config.rates.positionVelocityHz = kSparkStatusHz;
      config.rates.controlHz = kSparkControlHz;
    }
    else
    {
      const bool canFd = base::UseCanFdRates(config.canBus, false);
      config.rates.positionVelocityHz = canFd ? kTalonFdStatusHz : kTalonClassicStatusHz;
      config.rates.controlHz = canFd ? kTalonFdControlHz : kTalonClassicControlHz;
    }
    return config;
  }
  base::Gains JointGains(base::Gains gains)
  {
#ifndef __FRC_ROBORIO__
    if (base::CurrentRuntimeMode() == base::Mode::Sim)
      return simulation::ConfigureJointGains(gains);
#endif
    return gains;
  }
  const char *SystemCheckPath() { return base::CurrentRuntimeMode() == base::Mode::Real ? "/home/lvuser/systemcheck-latest.json" : "systemcheck-latest.json"; }
}
