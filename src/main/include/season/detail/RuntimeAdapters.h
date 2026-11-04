#pragma once

#include "season/shooting/Shooting.h"
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
}
