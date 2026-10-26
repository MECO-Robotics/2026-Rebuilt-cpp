#pragma once
#include "base/Drive.h"
#include <string>
namespace season
{
    // Loads at construction, snapshots alliance/reset odometry on initialize,
    // follows samples at 20ms with Java's translation P=10 and rotation P=7.
    frc2::CommandPtr Auto(base::Drive &drive, std::string trajectoryName);
}
