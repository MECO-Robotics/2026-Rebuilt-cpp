#pragma once
#include <units/time.h>

namespace season::hardware
{
  inline constexpr int kPilotControllerPort = 0, kCopilotControllerPort = 1;
  // Standard robot and command cadence. Increase only after validating loop
  // timing, motor IO, CAN load, and sensor transport as one system.
  inline constexpr units::second_t kControlPeriod{.02};
  // TODO: Increase control frequency after end-to-end timing and hardware validation.
}
