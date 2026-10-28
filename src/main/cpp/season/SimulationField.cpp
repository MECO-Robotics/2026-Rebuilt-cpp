#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include "base/sim/FootprintClearance.h"
#include <numbers>

namespace season::simulation {
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
