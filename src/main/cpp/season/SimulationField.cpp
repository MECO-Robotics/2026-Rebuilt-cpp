#ifndef __FRC_ROBORIO__
#include "season/Simulation.h"
#include "base/sim/FootprintClearance.h"
#include "base/sim/ColliderTerrain.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace season::simulation {
namespace {
const std::vector<base::sim::PitchedPlateTerrain>& Ramps() {
    static const auto ramps=base::sim::PitchedPlateTerrains(FieldColliders());
    return ramps;
}
}
std::vector<double> RampBoundaries(base::FootprintPose a,base::FootprintPose b) {
    return base::sim::TerrainBoundaries(Ramps(),a.x,a.y,b.x,b.y);
}
base::FootprintTerrainSegment RampTerrain(base::FootprintPose a,base::FootprintPose b) {
    return {base::sim::TerrainGrade(Ramps(),a.x,a.y,b.x,b.y),0};
}
std::vector<base::FieldRectangle> RampObstacles() {
    std::vector<base::FieldRectangle> boxes;
    for (const auto& ramp : Ramps())
        boxes.push_back({ramp.minX-.005,ramp.minY-.005,ramp.maxX+.005,ramp.maxY+.005});
    return boxes;
}
frc::Pose2d DefaultSpawn(bool red) {
    constexpr double laneY=units::meter_t{units::inch_t{50.34}}.value()/2;
    return {units::meter_t{red?kFieldLength-3.65:3.65},
            units::meter_t{red?kFieldWidth-laneY:laneY},
            frc::Rotation2d{units::radian_t{red?std::numbers::pi:0}}};
}
bool IsClearStartingPose(const frc::Pose2d& pose) {
    const auto colliders = FieldColliders();
    return base::sim::IsFootprintClear(pose.X().value(), pose.Y().value(),
        pose.Rotation().Radians().value(), .43, .43, .45,
        kFieldLength, kFieldWidth, .02, colliders);
}
void InstallField(base::sim::Arena& arena) {
    for (const auto& collider : FieldColliders()) arena.AddCollider(collider);
}
}
#endif
