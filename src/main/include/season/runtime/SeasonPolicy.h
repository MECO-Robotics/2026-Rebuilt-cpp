#pragma once

#include "season/shooting/Shooting.h"
#include <functional>

namespace season
{
  struct Mechanisms;

  namespace runtime
  {
    // Season behavior values selected for the current runtime mode.
    struct Behavior
    {
      double defaultCapacity = 0;
      double defaultFeedRate = 0;
      double emptyQuietSeconds = .4;
      double emptyShooterCurrentAmps = 0;
      double emptyRollerNoiseAmps = 0;
      double shotTimeoutSeconds = 15;
      std::function<bool(const Mechanisms &)> transportActive = [](const Mechanisms &) { return true; };
    };

    struct AutonomousDefaults
    {
      bool offensiveDefault;
      bool offenseEnabled;
    };

    struct PolicyProfile
    {
      AutonomousDefaults autonomous{false, false};
      ShotCalibration shot{};
      Behavior behavior{};
    };

    const PolicyProfile &SeasonPolicy();
  }
}
