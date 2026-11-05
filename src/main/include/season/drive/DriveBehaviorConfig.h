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

namespace season::drive::ntgamepadconfig
{
  // Accept axes published through the NT4 input topic.
  inline constexpr bool kEnabled = true;
  // Fall back to valid Driver Station axes if the NT stream is unavailable.
  inline constexpr bool kFallbackToDriverStation = true;
  inline constexpr double kDefaultPeriodMs = 5.0;
  inline constexpr double kDefaultStaleMs = 50.0;
}
