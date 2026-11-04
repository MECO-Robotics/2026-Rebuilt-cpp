#pragma once

// Driver and autonomous aiming settings.
namespace season::driveconfig
{
  // Joystick magnitude below this fraction is treated as zero by driver controls.
  inline constexpr double kDriverDeadband = .1;
  // Heading-error proportional gain used by manual aiming control.
  inline constexpr double kAimHeadingGain = 5;
  // Heading-error proportional gain used by autonomous aiming control.
  inline constexpr double kAutoAimHeadingGain = 7;
  // Maximum angular rate in rad/s commanded by autonomous aiming.
  inline constexpr double kAutoAimMaxAngularRate = 7;
}
