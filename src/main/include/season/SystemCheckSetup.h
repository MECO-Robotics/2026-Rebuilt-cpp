#pragma once
#include "base/SystemCheck.h"
#include "base/Mechanism.h"
#include "base/Drive.h"
#include <utility>

namespace season
{
    // Register unloaded pit checks only. Caller supplies ALL motors to neutralize in manager callback.
    // Shooter intentionally runs at 10 rps; no practice-shot/feed sequence is registered.
    void ConfigureSystemChecks(base::SystemCheck &check, base::Drive &drive,
                               base::PositionJoint &rack, base::PositionJoint &hood,
                               base::Flywheel &shooter,
                               std::vector<std::pair<std::string, base::Flywheel *>> rollers,
                               std::function<bool()> visionConnected);
}
