#pragma once

#include "season/detail/RuntimeAdapters.h"
#include "season/detail/RuntimeBehavior.h"

namespace season::runtime
{
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
