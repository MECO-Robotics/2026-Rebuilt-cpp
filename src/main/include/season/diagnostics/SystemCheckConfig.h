#pragma once
namespace season::diagnostics
{
  inline constexpr double kPreflightTimeoutSeconds = 3, kDriveTimeoutSeconds = 4;
  inline constexpr double kRackTimeoutSeconds = 5, kHoodTimeoutSeconds = 4;
  inline constexpr double kRollerTimeoutSeconds = 3, kShooterTimeoutSeconds = 5, kVisionTimeoutSeconds = 5;
  inline constexpr double kPreflightSettleSeconds = .25, kMotionSettleSeconds = .5;
  inline constexpr double kDriveSpeedMetresPerSecond = .5, kDriveToleranceMetresPerSecond = .15;
  inline constexpr double kRackTravelMetres = .02, kRackToleranceMetres = .01;
  inline constexpr double kHoodPositionRotations = .020, kHoodToleranceRotations = .003;
  inline constexpr double kRollerVoltage = 2, kRollerMinimumRps = 1;
  inline constexpr double kShooterRps = 10, kShooterToleranceRps = 2;
}
