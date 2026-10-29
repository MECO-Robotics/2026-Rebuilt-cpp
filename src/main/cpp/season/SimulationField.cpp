#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include "base/sim/FootprintClearance.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace season::simulation {
namespace {
struct Ramp { double xmin,xmax,ymin,ymax,gradeX; };
const std::vector<Ramp>& Ramps() {
    static const auto ramps=[] {
    std::vector<Ramp> ramps;
    for (const auto& b : FieldColliders()) {
        const auto q=b.pose.rotation;
        if (std::abs(q.x)>1e-8 || std::abs(q.z)>1e-8 || std::abs(q.y)<1e-8) continue;
        const double pitch=2*std::atan2(q.y,q.w);
        // Use the projected centerline of the plate; its thickness is not an
        // additional length of traversable slope at the crest.
        const double hx=std::abs(std::cos(pitch))*b.halfExtents.x;
        ramps.push_back({b.pose.position.x-hx,b.pose.position.x+hx,
            b.pose.position.y-b.halfExtents.y,b.pose.position.y+b.halfExtents.y,-std::tan(pitch)});
    }
    return ramps;
    }();
    return ramps;
}
bool Clip(double origin,double delta,double lo,double hi,double& enter,double& exit) {
    if (std::abs(delta)<1e-12) return origin>=lo && origin<=hi;
    double a=(lo-origin)/delta,b=(hi-origin)/delta;
    if (a>b) std::swap(a,b);
    enter=std::max(enter,a);exit=std::min(exit,b);
    return enter<exit;
}
}
std::vector<double> RampBoundaries(base::FootprintPose a,base::FootprintPose b) {
    std::vector<double> cuts;
    for (const auto& ramp : Ramps()) {
        double enter=0,exit=1;
        if (!Clip(a.x,b.x-a.x,ramp.xmin,ramp.xmax,enter,exit) ||
            !Clip(a.y,b.y-a.y,ramp.ymin,ramp.ymax,enter,exit)) continue;
        if (enter>1e-8 && enter<1-1e-8) cuts.push_back(enter);
        if (exit>1e-8 && exit<1-1e-8) cuts.push_back(exit);
    }
    std::sort(cuts.begin(),cuts.end());
    cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double a,double b) {return std::abs(a-b)<1e-7;}),cuts.end());
    return cuts;
}
base::FootprintTerrainSegment RampTerrain(base::FootprintPose a,base::FootprintPose b) {
    const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
    if (length<1e-9) return {};
    const double x=(a.x+b.x)/2,y=(a.y+b.y)/2;
    for (const auto& ramp : Ramps())
        if (x>=ramp.xmin && x<=ramp.xmax && y>=ramp.ymin && y<=ramp.ymax)
            return {ramp.gradeX*dx/length,0};
    return {};
}
std::vector<base::FieldRectangle> RampObstacles() {
    std::vector<base::FieldRectangle> boxes;
    for (const auto& ramp : Ramps())
        boxes.push_back({ramp.xmin-.005,ramp.ymin-.005,ramp.xmax+.005,ramp.ymax+.005});
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
