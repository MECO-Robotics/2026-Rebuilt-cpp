#pragma once
#ifndef __FRC_ROBORIO__
#include "base/sim/Arena.h"
#include "base/sim/fields/Rebuilt2026.h"
#include <frc/geometry/Pose2d.h>
#include <vector>

namespace season::simulation {
using base::sim::fields::rebuilt2026::kFieldLength;
using base::sim::fields::rebuilt2026::kFieldWidth;
using base::sim::fields::rebuilt2026::kBlueHubX;
using base::sim::fields::rebuilt2026::kRedHubX;
using base::sim::fields::rebuilt2026::kHubY;
using base::sim::fields::rebuilt2026::kStartingLineX;
using base::sim::fields::rebuilt2026::FieldColliders;
frc::Pose2d DefaultSpawn(bool red);
// Conservative clearance of the modeled .86m square chassis up to .45m high.
// This is a modeled start-space check, not whole-robot inspection acceptance.
bool IsClearStartingPose(const frc::Pose2d& pose);
void InstallField(base::sim::Arena& arena);
}
#endif
