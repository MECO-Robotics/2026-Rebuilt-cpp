#include "season/drive/OffensiveCollectionPlanner.h"
#include <frc/system/plant/DCMotor.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

using namespace units::literals;
namespace season::offense
{
  // Bound offensive pathfinding by the configured drivetrain and its low-speed
  // motor torque at the configured current limit. This is a theoretical ceiling;
  // the drivetrain plant, battery sag, and feedback controller may achieve less.
  base::AutoDriveConfig OffensiveMotionLimits(const base::DriveConfig &drive)
  {
    auto limits = base::AutoDriveConfig{};
    const auto motor = frc::DCMotor::KrakenX60FOC(1);
    const double current = std::min(drive.driveCurrent, drive.slipCurrent);
    if (drive.massKg > 0 && drive.wheelRadius > 0 && drive.driveRatio > 0 && current > 0)
    {
      const double wheelForce = 4 * motor.Kt.value() * current * drive.driveRatio / drive.wheelRadius;
      const double torqueLimitedAcceleration = wheelForce / drive.massKg;
      const double tractionLimitedAcceleration = drive.cof * 9.80665;
      const double electricalFreeSpeed = motor.freeSpeed.value() * drive.wheelRadius / drive.driveRatio;
      limits.maxSpeed = std::min(drive.maxSpeed, electricalFreeSpeed);
      limits.maxAcceleration = std::min(torqueLimitedAcceleration, tractionLimitedAcceleration);
    }
    return limits;
  }

  // Build a short, committed spline from field-frame observations. The route
  // contains at most the remaining estimated load, then points toward the next
  // action. PathPlanner's generated curve is checked against the same static
  // navgrid used by the obstacle-aware fallback before it is followed.
  std::shared_ptr<pathplanner::PathPlannerPath> CollectionSpline(
      frc::Pose2d start, const std::vector<base::ObjectCluster> &observations,
      frc::DriverStation::Alliance alliance, const FieldGeometry &field, const ScoringGrid &grid,
      bool active, bool ferry, int remaining, const base::AutoDriveConfig &limits)
  {
    const double maxSpeed = limits.maxSpeed;
    if (remaining <= 0 || !std::isfinite(maxSpeed) || maxSpeed <= 0 || grid.cellSize <= 0 || grid.blocked.empty())
      return {};
    auto free = [&grid](frc::Translation2d p)
    {
      return base::IsNavigationGridPointFree(grid, p);
    };
    if (!free(start.Translation()))
      return {};
    const bool home = InAllianceZone(alliance, start.Translation(), field);
    if (!active && home)
      return {}; // do not reacquire ferried fuel during the inactive shift
    std::vector<frc::Translation2d> candidates;
    candidates.reserve(observations.size());
    for (const auto &observation : observations)
      if (std::isfinite(observation.position.X().value()) && std::isfinite(observation.position.Y().value()) &&
          InAllianceZone(alliance, observation.position, field) == home && free(observation.position))
        candidates.push_back(observation.position);
    std::vector<frc::Translation2d> points{start.Translation()};
    frc::Rotation2d direction = start.Rotation();
    const int limit = std::min(remaining, offenseconfig::kMaxCollectionCandidates); // bound planner work per scheduler cycle
    while (!candidates.empty() && static_cast<int>(points.size()) - 1 < limit)
    {
      auto best = candidates.end();
      double bestCost = std::numeric_limits<double>::infinity();
      for (auto it = candidates.begin(); it != candidates.end(); ++it)
      {
        const auto delta = *it - points.back();
        const double distance = delta.Norm().value();
        if (distance < offenseconfig::kCollectionMinSeparation || distance > offenseconfig::kCollectionMaxSeparation)
          continue;
        const double turn = std::abs(std::remainder((delta.Angle() - direction).Radians().value(), 2 * std::numbers::pi));
        if (points.size() > 1 && turn > offenseconfig::kCollectionMaxTurnRadians)
          continue;
        const double cost = distance + (active ? offenseconfig::kActiveTurnWeight : offenseconfig::kInactiveTurnWeight) * turn;
        if (cost < bestCost)
        {
          best = it;
          bestCost = cost;
        }
      }
      if (best == candidates.end())
        break;
      direction = (*best - points.back()).Angle();
      const auto selected = *best;
      points.push_back(selected);
      std::erase_if(candidates, [&](const auto &p)
                    { return p.Distance(selected).value() < offenseconfig::kCollectionMinSeparation; });
    }
    if (points.size() < 3)
      return {};
    const size_t collectionPoints = points.size();
    // Add a scoring/ferry approach only when the visible chain accounts for
    // the remaining estimated load. Otherwise replan for more observations.
    if (static_cast<int>(collectionPoints) - 1 >= remaining)
    {
      const double sign = alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
      const double hub = HubPosition(alliance).X().value();
      const double lane = points.back().Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
      if (!home)
      {
        const frc::Translation2d neutral{units::meter_t{hub + sign * offenseconfig::kHubCrossingOffset}, units::meter_t{lane}};
        if (points.back().Distance(neutral).value() > offenseconfig::kCollectionMinSeparation)
          points.push_back(neutral);
        // While inactive, stop on the neutral side; the stage/ferry command
        // owns the subsequent action and decides when acquisition stops.
        if (active && !ferry)
          points.emplace_back(units::meter_t{hub - sign * offenseconfig::kHubCrossingOffset}, units::meter_t{lane});
      }
    }
    const auto &constraints = limits;
    auto create = [&](size_t count) -> std::shared_ptr<pathplanner::PathPlannerPath>
    {
      try
      {
        std::vector<frc::Pose2d> poses;
        std::vector<pathplanner::RotationTarget> rotations;
        poses.reserve(count);
        rotations.reserve(count);
        for (size_t i = 0; i < count; ++i)
        {
          const auto tangent = i + 1 < count ? (points[i + 1] - points[i]).Angle() : (points[i] - points[i - 1]).Angle();
          poses.emplace_back(points[i], tangent);
          if (i > 0 && i + 1 < count)
            rotations.emplace_back(static_cast<double>(i), tangent);
        }
        auto path = std::make_shared<pathplanner::PathPlannerPath>(
            pathplanner::PathPlannerPath::waypointsFromPoses(poses), rotations,
            std::vector<pathplanner::PointTowardsZone>{}, std::vector<pathplanner::ConstraintsZone>{},
            std::vector<pathplanner::EventMarker>{},
            pathplanner::PathConstraints{units::meters_per_second_t{std::min(maxSpeed, constraints.maxSpeed)},
                                         units::meters_per_second_squared_t{constraints.maxAcceleration}, units::radians_per_second_t{constraints.maxOmega},
                                         units::radians_per_second_squared_t{constraints.maxAlpha}},
            std::nullopt,
            pathplanner::GoalEndState{0_mps, poses.back().Rotation()}, false);
        path->preventFlipping = true; // observations and route are already field-relative on both alliances
        const auto &samples = path->getAllPathPoints();
        if (samples.size() < 2)
          return {};
        const auto clear = [&](frc::Translation2d p)
        {
          const frc::Pose2d pose{p, frc::Rotation2d{}};
          return free(p) && RobotClearOfBumps(pose) && RobotClearOfDepot(pose, alliance, field);
        };
        for (size_t i = 0; i < samples.size(); ++i)
        {
          if (!clear(samples[i].position))
            return {};
          if (i == 0)
            continue;
          const auto d = samples[i].position - samples[i - 1].position;
          const int steps = std::max(1, static_cast<int>(std::ceil(d.Norm().value() / (grid.cellSize / 4.))));
          for (int j = 1; j < steps; ++j)
            if (!clear(samples[i - 1].position + d * (static_cast<double>(j) / steps)))
              return {};
        }
        return path;
      }
      catch (const std::exception &)
      {
        return {};
      }
    };
    // If the exit crosses a blocked cell, retain the collection chain and let
    // the existing pathfinder plan the separate return or ferry approach.
    if (points.size() > collectionPoints)
      if (auto path = create(points.size()))
        return path;
    for (size_t count = collectionPoints; count >= 3; --count)
      if (auto path = create(count))
        return path;
    return {};
  }
} // namespace season::offense
