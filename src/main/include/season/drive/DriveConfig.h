#pragma once
#include "base/Drive.h"
#include "season/RobotControlConfig.h"

namespace season::hardware
{
  // Requested odometry sampling rates by CAN network type, in hertz.
  inline constexpr double kSwerveOdometryFdHz = 500, kSwerveOdometryClassicHz = 100;
  using namespace units::literals;
  inline base::DriveConfig HardwareDriveConfig(double odometryHz)
  {
    base::DriveConfig c;
    c.bus = "MECO CANIvore";
    c.pigeon = 13;
    c.odometryHz = odometryHz;
    c.controlPeriodSeconds = kControlPeriod.value();
    c.driveRatio = 5.8909090909090915;
    c.steerRatio = 12.1;
    c.couplingRatio = 4.909090909090909;
    c.maxSpeed = 5.24;
    c.maxAngularRate = 7;
    c.driveKP = .15;
    c.driveKS = .1;
    c.driveKV = .124;
    c.steerKP = 100;
    c.steerKD = .5;
    c.steerKS = .2;
    c.steerKV = 1.5;
    c.driveCurrent = 55;
    c.slipCurrent = 50;
    c.steerCurrent = 30;
    // Replace the yaw MOI estimate when a measured or CAD value is available.
    c.massKg = 53;
    c.moi = 6.883;
    c.cof = 2.9;
    c.autoConfigFromGUI = true;
    c.autoTranslationKP = 15;
    c.autoRotationKP = 10;
    // Module order is front-left, front-right, back-left, back-right.
    c.modules = {{
        {.drive = 2, .steer = 1, .encoder = 9, .offsetTurns = .3076171875,
         .xMeters = .2794, .yMeters = .2794,
         .driveInverted = false, .steerInverted = true, .encoderInverted = false},
        {.drive = 8, .steer = 3, .encoder = 10, .offsetTurns = .304931640625,
         .xMeters = .2794, .yMeters = -.2794,
         .driveInverted = true, .steerInverted = true, .encoderInverted = false},
        {.drive = 6, .steer = 5, .encoder = 11, .offsetTurns = -.083740234375,
         .xMeters = -.2794, .yMeters = .2794,
         .driveInverted = false, .steerInverted = true, .encoderInverted = false},
        {.drive = 4, .steer = 7, .encoder = 12, .offsetTurns = -.346435546875,
         .xMeters = -.2794, .yMeters = -.2794,
         .driveInverted = true, .steerInverted = true, .encoderInverted = false}}};
    return c;
  }
  // Selects platform-specific configuration before the drivetrain is constructed.
  base::DriveConfig DriveConfig();
}
