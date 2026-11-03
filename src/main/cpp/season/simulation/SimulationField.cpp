#include "season/simulation/RuntimeConfig.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "base/sim/FootprintClearance.h"
#include "base/sim/ColliderTerrain.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace season::simulation
{
  namespace
  {
    const std::vector<base::sim::PitchedPlateTerrain> &Ramps()
    {
      static const auto ramps = base::sim::PitchedPlateTerrains(FieldColliders());
      return ramps;
    }
  } // namespace
  std::vector<double> RampBoundaries(base::FootprintPose a, base::FootprintPose b)
  {
    return base::sim::TerrainBoundaries(Ramps(), a.x, a.y, b.x, b.y);
  }
  base::FootprintTerrainSegment RampTerrain(base::FootprintPose a, base::FootprintPose b)
  {
    return {base::sim::TerrainGrade(Ramps(), a.x, a.y, b.x, b.y), 0};
  }
  std::vector<base::FieldRectangle> RampObstacles()
  {
    std::vector<base::FieldRectangle> boxes;
    for (const auto &ramp : Ramps())
      boxes.push_back({ramp.minX - runtimeconfig::kRampClearancePaddingMetres, ramp.minY - runtimeconfig::kRampClearancePaddingMetres, ramp.maxX + runtimeconfig::kRampClearancePaddingMetres, ramp.maxY + runtimeconfig::kRampClearancePaddingMetres});
    return boxes;
  }
  // Mirror the clear trench-lane spawn for red.
  frc::Pose2d DefaultSpawn(bool red)
  {
    constexpr double laneY = units::meter_t{units::inch_t{runtimeconfig::kSpawnLaneWidthInches}}.value() / 2;
    return {units::meter_t{red ? kFieldLength - runtimeconfig::kSpawnXMetres : runtimeconfig::kSpawnXMetres}, units::meter_t{red ? kFieldWidth - laneY : laneY},
            frc::Rotation2d{units::radian_t{red ? std::numbers::pi : 0}}};
  }
  // Reject starts that intersect modeled field geometry.
  bool IsClearStartingPose(const frc::Pose2d &pose)
  {
    const auto colliders = FieldColliders();
    return base::sim::IsFootprintClear(pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value(), runtimeconfig::kFootprintHalfLengthMetres, runtimeconfig::kFootprintHalfWidthMetres,
                                       runtimeconfig::kFootprintHeightMetres, kFieldLength, kFieldWidth, runtimeconfig::kSpawnClearanceMetres, colliders);
  }
  // Install the season collision bodies into the simulation arena.
  void InstallField(base::sim::Arena &arena)
  {
    for (const auto &collider : FieldColliders())
      arena.AddCollider(collider);
  }
} // namespace season::simulation
#endif
