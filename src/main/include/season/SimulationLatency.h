#pragma once

// Opt-in desktop measurement only. No changes to control or simulation timing.
#ifndef __FRC_ROBORIO__
#include <algorithm>
#include <array>
#include "base/Telemetry.h"
#include <cstdlib>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
namespace season::simulation {
inline bool LatencyProbeEnabled() {
  static const bool enabled = std::getenv("FRC_SIM_LATENCY_PROBE") != nullptr;
  return enabled;
}
inline std::array<double, 6> latencySample{};
inline double ObserveJoystick(double value) {
  if (LatencyProbeEnabled()) {
    latencySample[3]=value;
    latencySample[4]=nt::Now();
  }
  return value;
}
inline void BeginLatencyProbe() {
  if (!LatencyProbeEnabled()) return;
  latencySample[0]=nt::Now();
  latencySample[2]=-frc::DriverStation::GetStickAxis(0,1);
}
inline void EndLatencyProbe() {
  if (!LatencyProbeEnabled()) return;
  latencySample[1]=nt::Now();
  latencySample[5]=frc::RobotController::GetFPGATime();
  static auto publisher=nt::NetworkTableInstance::GetDefault()
      .GetDoubleArrayTopic("/Diagnostics/Latency/RobotLoop").Publish();
  publisher.Set(latencySample);
  auto captured=latencySample;
  base::telemetry::Post("LatencyProbe", [captured] {
    std::array<double,7> queued{};
    std::copy(captured.begin(),captured.end(),queued.begin());
    queued[6]=nt::Now();
    static auto background=nt::NetworkTableInstance::GetDefault()
        .GetDoubleArrayTopic("/Diagnostics/Latency/Background").Publish();
    background.Set(queued);
  });
}
}
#else
namespace season::simulation {
inline double ObserveJoystick(double value) { return value; }
inline void BeginLatencyProbe() {}
inline void EndLatencyProbe() {}
}
#endif
