#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace season::simulation {
frc::Pose2d DefaultSpawn(bool red) {
    constexpr double laneY=units::meter_t{units::inch_t{50.34}}.value()/2;
    return {units::meter_t{red?kFieldLength-3.65:3.65},
            units::meter_t{red?kFieldWidth-laneY:laneY},
            frc::Rotation2d{units::radian_t{red?std::numbers::pi:0}}};
}
bool IsClearStartingPose(const frc::Pose2d& pose) {
    const double x=pose.X().value(), y=pose.Y().value(), a=pose.Rotation().Radians().value();
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(a)) return false;
    const double c=std::cos(a),s=std::sin(a),r=.43*(std::abs(c)+std::abs(s));
    if (x-r<=0||x+r>=kFieldLength||y-r<=0||y+r>=kFieldWidth) return false;
    // SAT on XY projections; sloped plates conservatively use their bounding box.
    for (const auto& b : FieldColliders()) {
        const auto& q=b.pose.rotation;
        const double pitch=2*std::atan2(q.y,q.w);
        const double ex=std::abs(std::cos(pitch))*b.halfExtents.x+
                        std::abs(std::sin(pitch))*b.halfExtents.z;
        const double ez=std::abs(std::sin(pitch))*b.halfExtents.x+
                        std::abs(std::cos(pitch))*b.halfExtents.z;
        if (b.pose.position.z-ez>.45) continue;
        const double dx=x-b.pose.position.x,dy=y-b.pose.position.y,ey=b.halfExtents.y;
        bool separated=false;
        for (const auto& axis : std::array<std::array<double,2>,4>{{{1,0},{0,1},{c,s},{-s,c}}}) {
            const double u=axis[0],v=axis[1];
            const double robot=.43*(std::abs(u*c+v*s)+std::abs(-u*s+v*c));
            const double obstacle=ex*std::abs(u)+ey*std::abs(v);
            if (std::abs(dx*u+dy*v)>robot+obstacle+.02) separated=true;
        }
        if (!separated) return false;
    }
    return true;
}
void InstallField(base::sim::Arena& arena) {
    for (const auto& collider : FieldColliders()) arena.AddCollider(collider);
}
}
#endif
