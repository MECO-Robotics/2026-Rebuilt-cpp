#pragma once
#include <units/time.h>

namespace season::hardware
{
  // USB ports assigned to the pilot and copilot controllers, respectively.
  inline constexpr int kPilotControllerPort = 0, kCopilotControllerPort = 1;
  // Standard robot and command cadence. Increase only after validating loop
  // timing, motor IO, CAN load, and sensor transport as one system.
  inline constexpr double kRobotControlFrequencyHz = 50;
  inline constexpr units::second_t kControlPeriod{1.0 / kRobotControlFrequencyHz};
  // TODO: Increase control frequency after end-to-end timing and hardware validation.
}
