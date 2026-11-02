#pragma once

namespace season::drive::fastcontrol
{
// Runtime opt-in command cadence; the TimedRobot and slow simulation callbacks remain at 20 ms.
inline constexpr double kPeriodSeconds = 0.002;
inline constexpr double kPeriodMicroseconds = 2000.0;
} // namespace season::drive::fastcontrol
