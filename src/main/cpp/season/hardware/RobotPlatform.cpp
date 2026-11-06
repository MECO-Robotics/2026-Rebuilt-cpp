#include "season/hardware/RobotPlatform.h"
#include "season/drive/DriveConfig.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/DriveSimulationConfig.h"
#endif
#include "season/mechanisms/Mechanisms.h"
#include <base/CanBusMode.h>
#include <base/RuntimeMode.h>
#include <utility>
namespace season::hardware {
  base::DriveConfig DriveConfig()
  {
    auto config = HardwareDriveConfig(kSwerveOdometryFdHz);
    config.odometryHz = base::SelectCanBusRate(config.bus, kSwerveOdometryFdHz, kSwerveOdometryClassicHz);
#ifndef __FRC_ROBORIO__
    return base::ConfigureRuntimeDrive(std::move(config), [](base::DriveConfig c) {
      c.odometryHz = kSwerveOdometryFdHz;
      return simulation::ConfigureDrive(std::move(c), kSimulationTuning);
    });
#else
    return base::ConfigureRuntimeDrive(std::move(config));
#endif
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
    return base::ConfigureRuntimeJointGains(gains, kSimulationTuning, base::SimulationJointGravity::Unmodeled);
  }
  const char *SystemCheckPath() { return base::CurrentRuntimeMode() == base::Mode::Real ? "/home/lvuser/systemcheck-latest.json" : "systemcheck-latest.json"; }
}
