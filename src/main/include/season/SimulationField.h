#pragma once
#ifndef __FRC_ROBORIO__
#include "base/sim/Arena.h"
#include "base/sim/fields/Rebuilt2026.h"
#include "base/FootprintSpeedProfile.h"
#include <frc/geometry/Pose2d.h>
#include <vector>

namespace season::simulation
{
    using base::sim::fields::rebuilt2026::FieldColliders;
    using base::sim::fields::rebuilt2026::kBlueHubX;
    using base::sim::fields::rebuilt2026::kFieldLength;
    using base::sim::fields::rebuilt2026::kFieldWidth;
    using base::sim::fields::rebuilt2026::kHubY;
    using base::sim::fields::rebuilt2026::kRedHubX;
    using base::sim::fields::rebuilt2026::kStartingLineX;
    frc::Pose2d DefaultSpawn(bool red);
    // Conservative clearance of the modeled .86m square chassis up to .45m high.
    // This is a modeled start-space check, not whole-robot inspection acceptance.
    bool IsClearStartingPose(const frc::Pose2d &pose);
    void InstallField(base::sim::Arena &arena);
    // Centerline terrain for the modeled pitched plates; rolling resistance is
    // not modeled here. These functions are desktop simulation inputs only.
    std::vector<double> RampBoundaries(base::FootprintPose a, base::FootprintPose b);
    base::FootprintTerrainSegment RampTerrain(base::FootprintPose a, base::FootprintPose b);
    // Conservative plan-view rectangles for the optional ramp avoidance policy.
    std::vector<base::FieldRectangle> RampObstacles();
}
#endif
