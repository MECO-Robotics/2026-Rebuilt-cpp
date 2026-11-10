#pragma once
#include "base/Drive.h"
#include "season/RobotControlConfig.h"
#include "season/RobotGeometry.h"

namespace season::hardware
{
  // Requested odometry sampling rates by CAN network type, in hertz.
  inline constexpr double kSwerveOdometryFdHz = 500, kSwerveOdometryClassicHz = 100;
  inline constexpr int kPigeonCanId = 13;
  inline constexpr int kFrontLeftDriveCanId = 2, kFrontLeftSteerCanId = 1, kFrontLeftEncoderCanId = 9;
  inline constexpr int kFrontRightDriveCanId = 8, kFrontRightSteerCanId = 3, kFrontRightEncoderCanId = 10;
  inline constexpr int kBackLeftDriveCanId = 6, kBackLeftSteerCanId = 5, kBackLeftEncoderCanId = 11;
  inline constexpr int kBackRightDriveCanId = 4, kBackRightSteerCanId = 7, kBackRightEncoderCanId = 12;

  // Preserve the configured drive reduction as a rational compile-time expression.
  inline constexpr double kDriveRatio = 324.0 / 55.0;
  inline constexpr double kSteerRatio = 121.0 / 10.0;
  inline constexpr double kCouplingRatio = 54.0 / 11.0;

  // Four-inch tread diameter and 22-inch wheelbase/track width.
  inline constexpr double kWheelDiameterMetres = 4.0 * season::kMetresPerInch;
  inline constexpr double kWheelRadiusMetres = kWheelDiameterMetres / 2;
  inline constexpr double kHalfWheelbaseMetres = 22.0 * season::kMetresPerInch / 2;
  inline constexpr double kHalfTrackWidthMetres = 22.0 * season::kMetresPerInch / 2;

  // Drivetrain limits, feedforward, feedback, and current limits.
  inline constexpr double kMaxDriveSpeedMetresPerSecond = 5.24;
  inline constexpr double kMaxAngularRateRadiansPerSecond = 7;
  inline constexpr double kDriveKp = .15, kDriveKsVolts = .1, kDriveKvVoltsPerMetrePerSecond = .124;
  inline constexpr double kSteerKp = 100, kSteerKd = .5, kSteerKsVolts = .2, kSteerKvVoltsPerRadianPerSecond = 1.5;
  inline constexpr double kDriveCurrentLimitAmps = 55, kSlipCurrentAmps = 50, kSteerCurrentLimitAmps = 30;

  // Robot mass properties and traction estimate.
  inline constexpr double kRobotMassKg = 53;
  inline constexpr double kRobotYawMomentOfInertiaKgMetresSquared = 6.883;
  inline constexpr double kWheelCoefficientOfFriction = 2.9;

  // Autonomous path follower gains.
  inline constexpr double kAutoTranslationKp = 15, kAutoRotationKp = 10;

  // Measured absolute encoder zero positions, as fractions of a turn.
  inline constexpr double kCANcoderCountsPerTurn = 65536.0;
  inline constexpr double kFrontLeftEncoderOffsetTurns = 20160.0 / kCANcoderCountsPerTurn;
  inline constexpr double kFrontRightEncoderOffsetTurns = 19984.0 / kCANcoderCountsPerTurn;
  inline constexpr double kBackLeftEncoderOffsetTurns = -5488.0 / kCANcoderCountsPerTurn;
  inline constexpr double kBackRightEncoderOffsetTurns = -22704.0 / kCANcoderCountsPerTurn;

  using namespace units::literals;
  inline base::DriveConfig HardwareDriveConfig(double odometryHz)
  {
    base::DriveConfig c;
    c.bus = "MECO CANIvore";
    c.pigeon = kPigeonCanId;
    c.odometryHz = odometryHz;
    c.controlPeriodSeconds = kControlPeriod.value();
    c.driveRatio = kDriveRatio;
    c.steerRatio = kSteerRatio;
    c.couplingRatio = kCouplingRatio;
    c.wheelRadius = kWheelRadiusMetres;
    c.maxSpeed = kMaxDriveSpeedMetresPerSecond;
    c.maxAngularRate = kMaxAngularRateRadiansPerSecond;
    c.driveKP = kDriveKp;
    c.driveKS = kDriveKsVolts;
    c.driveKV = kDriveKvVoltsPerMetrePerSecond;
    c.steerKP = kSteerKp;
    c.steerKD = kSteerKd;
    c.steerKS = kSteerKsVolts;
    c.steerKV = kSteerKvVoltsPerRadianPerSecond;
    c.driveCurrent = kDriveCurrentLimitAmps;
    c.slipCurrent = kSlipCurrentAmps;
    c.steerCurrent = kSteerCurrentLimitAmps;
    c.backend = base::DriveBackend::Phoenix;
    // Replace the yaw MOI estimate when a measured or CAD value is available.
    c.massKg = kRobotMassKg;
    c.moi = kRobotYawMomentOfInertiaKgMetresSquared;
    c.cof = kWheelCoefficientOfFriction;
    c.autoConfigFromGUI = true;
    c.autoTranslationKP = kAutoTranslationKp;
    c.autoRotationKP = kAutoRotationKp;
    // Module order is front-left, front-right, back-left, back-right.
    c.modules = {{
        {.drive = kFrontLeftDriveCanId, .steer = kFrontLeftSteerCanId, .encoder = kFrontLeftEncoderCanId,
         .offsetTurns = kFrontLeftEncoderOffsetTurns,
         .xMeters = kHalfWheelbaseMetres, .yMeters = kHalfTrackWidthMetres,
         .driveInverted = false, .steerInverted = true, .encoderInverted = false},
        {.drive = kFrontRightDriveCanId, .steer = kFrontRightSteerCanId, .encoder = kFrontRightEncoderCanId,
         .offsetTurns = kFrontRightEncoderOffsetTurns,
         .xMeters = kHalfWheelbaseMetres, .yMeters = -kHalfTrackWidthMetres,
         .driveInverted = true, .steerInverted = true, .encoderInverted = false},
        {.drive = kBackLeftDriveCanId, .steer = kBackLeftSteerCanId, .encoder = kBackLeftEncoderCanId,
         .offsetTurns = kBackLeftEncoderOffsetTurns,
         .xMeters = -kHalfWheelbaseMetres, .yMeters = kHalfTrackWidthMetres,
         .driveInverted = false, .steerInverted = true, .encoderInverted = false},
        {.drive = kBackRightDriveCanId, .steer = kBackRightSteerCanId, .encoder = kBackRightEncoderCanId,
         .offsetTurns = kBackRightEncoderOffsetTurns,
         .xMeters = -kHalfWheelbaseMetres, .yMeters = -kHalfTrackWidthMetres,
         .driveInverted = true, .steerInverted = true, .encoderInverted = false}}};
    return c;
  }
  // Selects platform-specific configuration before the drivetrain is constructed.
  base::DriveConfig DriveConfig();
}
