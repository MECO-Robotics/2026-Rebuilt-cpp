#pragma once
namespace season::diagnostics
{
  // Maximum times allowed for preflight and drive checks, respectively.
  inline constexpr double kPreflightTimeoutSeconds = 3, kDriveTimeoutSeconds = 4;
  // Maximum times allowed for rack and hood motion checks, respectively.
  inline constexpr double kRackTimeoutSeconds = 5, kHoodTimeoutSeconds = 4;
  // Maximum times allowed for roller, shooter, and vision checks, respectively.
  inline constexpr double kRollerTimeoutSeconds = 3, kShooterTimeoutSeconds = 5, kVisionTimeoutSeconds = 5;
  // Settling delays after preflight commands and motion commands, respectively.
  inline constexpr double kPreflightSettleSeconds = .25, kMotionSettleSeconds = .5;
  // Requested chassis speed and accepted speed error during the drive check.
  inline constexpr double kDriveSpeedMetresPerSecond = .5, kDriveToleranceMetresPerSecond = .15;
  // Requested rack displacement and accepted final position error during its motion check.
  inline constexpr double kRackTravelMetres = .02, kRackToleranceMetres = .01;
  // Hood target and accepted final position error during its motion check.
  inline constexpr double kHoodPositionRotations = .020, kHoodToleranceRotations = .003;
  // Applied roller voltage and minimum speed considered a successful response.
  inline constexpr double kRollerVoltage = 2, kRollerMinimumRps = 1;
  // Shooter target speed and accepted speed error during its check.
  inline constexpr double kShooterRps = 10, kShooterToleranceRps = 2;
}
