#pragma once

#include "season/shooting/Shooting.h"
#include "season/detail/RuntimeBehavior.h"
#include <frc2/command/CommandPtr.h>
#include <optional>

namespace season::runtime
{
  // Apply any platform-specific input observation before driver shaping.
  double ObserveForwardInput(double value);

  struct AutonomousDefaults
  {
    bool offensiveDefault;
    bool offenseEnabled;
  };
  AutonomousDefaults AutoDefaults();
  ShotCalibration ShootingCalibration();

  // Some platforms provide an additional practice control for the dashboard.
  std::optional<frc2::CommandPtr> PracticeControl();

  // Game policy values are composed once. Base selects the matching profile
  // without making robot behavior or its callers inspect the runtime mode.
  struct PolicyProfile
  {
    AutonomousDefaults autonomous{false, false};
    ShotCalibration shot{};
    Behavior behavior{};
  };
  const PolicyProfile &Policy();
}
