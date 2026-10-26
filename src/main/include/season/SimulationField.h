#pragma once
#ifndef __FRC_ROBORIO__
#include "base/sim/Arena.h"
#include <frc/geometry/Pose2d.h>
#include <vector>

namespace season::simulation {
// WPILib 2026.2.1 welded-field, fixed blue-origin coordinates, metres.
inline constexpr double kFieldLength = 16.541;
inline constexpr double kFieldWidth = 8.069;
inline constexpr double kBlueHubX = 4.6256194;
inline constexpr double kRedHubX = 11.9154194;
inline constexpr double kHubY = 4.0346376;
inline constexpr double kStartingLineX = kBlueHubX - .5969;
std::vector<base::sim::BodySpec> FieldColliders();
frc::Pose2d DefaultSpawn(bool red);
// Conservative clearance of the modeled .86m square chassis up to .45m high.
// This is a modeled start-space check, not whole-robot inspection acceptance.
bool IsClearStartingPose(const frc::Pose2d& pose);
void InstallField(base::sim::Arena& arena);
}
#endif
