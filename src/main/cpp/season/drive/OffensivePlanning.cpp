#include "season/drive/OffensiveAutoDrive.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <frc/Timer.h>
#include <numbers>

namespace season
{
bool OffensiveHubShotAllowed(std::optional<bool> hubActive, std::optional<frc::Pose2d> target, frc::Pose2d pose,
                                      frc::ChassisSpeeds speeds)
  {
    return hubActive.value_or(false) && target && AutoDriveArrived(pose, speeds, *target);

  }

bool OffensiveFerryWindowClosing(std::optional<double> nextChange)
  {
    return nextChange && std::isfinite(*nextChange) && *nextChange >= 0. && *nextChange <= offenseconfig::kFerryClosingSeconds;

  }

std::optional<frc::Translation2d> MatchOffensiveTarget(frc::Translation2d locked,
                                                                const std::vector<base::ObjectCluster> &points)
  {
    const auto nearest = std::min_element(points.begin(), points.end(), [&](const auto &a, const auto &b)
                                          { return a.position.Distance(locked) < b.position.Distance(locked); });
    if (nearest == points.end() || nearest->position.Distance(locked).value() >= offenseconfig::kPickupMatchDistance)
      return std::nullopt;
    return nearest->position;

  }

OffensiveCommitment ReconcileOffensiveCommitment(OffensiveCommitment current, std::optional<bool> hubActive,
                                                          bool fullFeedCompleted)
  {
    if (fullFeedCompleted)
      return OffensiveCommitment::None;
    if (!hubActive)
      return current;
    if ((current == OffensiveCommitment::Score && !*hubActive) || (current == OffensiveCommitment::Ferry && *hubActive))
      return OffensiveCommitment::None;
    return current;

  }

bool OffensiveScoringWindowClosing(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                                            const FieldGeometry &field, std::optional<double> nextChange)
  {
    if (!nextChange || !std::isfinite(*nextChange) || *nextChange < 0)
      return false;
    const bool red = alliance == frc::DriverStation::Alliance::kRed;
    const double laneY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
    const frc::Translation2d home{units::meter_t{red ? field.length - offenseconfig::kReturnHomeXMetres : offenseconfig::kReturnHomeXMetres},
                                  units::meter_t{laneY}};
    const frc::Translation2d trench{units::meter_t{red ? field.length - offenseconfig::kReturnTrenchXMetres : offenseconfig::kReturnTrenchXMetres},
                                    units::meter_t{laneY}};
    const double route = InAllianceZone(alliance, pose.Translation(), field)
                             ? pose.Translation().Distance(home).value()
                             : pose.Translation().Distance(trench).value() + trench.Distance(home).value();
    // Reserve time for path setup, settling, aiming, simulated transport
    // reversal, and a volley after reaching the alliance zone.
    return *nextChange <= route + offenseconfig::kReturnSetupSeconds;

  }

OffensiveDecision DecideOffense(std::optional<bool> active, std::optional<frc::DriverStation::Alliance> alliance,
                                         frc::Pose2d pose, const FieldGeometry &field,
                                         const std::vector<base::ObjectCluster> &points, int estimatedFuel,
                                         double capacity, bool scoreCommitted, bool ferryCommitted,
                                         bool autonomous, std::optional<double> periodTime,
                                         bool collectionExpired, bool loadUnknown,
                                         std::optional<frc::Translation2d> retainedPickup)
  {
    if (!active || !alliance)
      return OffensiveDecision::Wait;
    const bool inZone = InAllianceZone(*alliance, pose.Translation(), field);
    const bool capacityKnown = std::isfinite(capacity) && capacity > 0;
    const bool full = capacityKnown && estimatedFuel >= capacity;
    // A temporary loss of camera observations does not establish that the
    // field is empty; continue the bounded search instead of returning early.
    (void)points;
    (void)retainedPickup;
    const bool autoReturn = autonomous && periodTime && std::isfinite(*periodTime) && *periodTime <= offenseconfig::kAutonomousReturnSeconds;
    // No possession sensor: zero inferred pickups is unknown, not proof of an
    // empty hopper. Timed attempts may therefore operate on an empty hopper.
    const bool mayCarry = estimatedFuel > 0 || loadUnknown;
    // Ferry trajectories must cross into the alliance zone from the neutral side.
    // Keep collecting toward neutral rather than committing to an impossible shot
    // while the robot is still in its own zone.
    if (!*active && inZone)
      return OffensiveDecision::Collect;
    if (mayCarry && *active &&
        (autoReturn || collectionExpired ||
         (estimatedFuel > 0 && (scoreCommitted || full))))
      return OffensiveDecision::Score;
    if (mayCarry && !*active && (collectionExpired || (estimatedFuel > 0 && (ferryCommitted || full))))
      return OffensiveDecision::Ferry;
    return OffensiveDecision::Collect;

  }

void OffensiveFuelEstimate::Observe(const std::vector<base::ObjectCluster> &points, frc::Pose2d pose)
  {
      const double now = frc::Timer::GetFPGATimestamp().value();
      for (auto &track : tracks_)
        track.seen = false;
      for (const auto &point : points)
      {
        const auto local = (point.position - pose.Translation()).RotateBy(-pose.Rotation());
        // The front camera is ahead of the intake while deployed. Remember
        // observations before they pass behind that camera's view.
        if (local.X().value() < offenseconfig::kObserveMinX || local.X().value() > offenseconfig::kObserveMaxX || std::abs(local.Y().value()) > offenseconfig::kObserveMaxAbsY)
          continue;
        auto nearest = std::min_element(tracks_.begin(), tracks_.end(), [&](const auto &a, const auto &b)
                                        { return a.position.Distance(point.position) < b.position.Distance(point.position); });
        if (nearest == tracks_.end() || nearest->position.Distance(point.position).value() >= offenseconfig::kTrackMatchDistance)
          tracks_.push_back({point.position, now, true, false, false});
        else
        {
          nearest->position = point.position;
          nearest->lastSeen = now;
          nearest->seen = true;
        }
      }
      for (auto &track : tracks_)
      {
        const auto local = (track.position - pose.Translation()).RotateBy(-pose.Rotation());
        const bool mouth = local.X().value() >= offenseconfig::kMouthMinX && local.X().value() <= offenseconfig::kMouthMaxX && std::abs(local.Y().value()) <= offenseconfig::kMouthHalfWidth;
        track.enteredMouth |= mouth;
        // Fresh camera frames can retain a pickup briefly after it passes the
        // mouth. Keep the crossing evidence until the track leaves our local
        // retention region instead of requiring disappearance at the mouth.
        if (!track.seen && !track.consumed && track.enteredMouth && now - track.lastSeen <= offenseconfig::kTrackRetentionSeconds &&
            local.X().value() >= offenseconfig::kTrackMinX && local.X().value() <= offenseconfig::kMouthMaxX && std::abs(local.Y().value()) <= offenseconfig::kMouthHalfWidth)
        {
          ++count_;
          track.consumed = true;
          track.lastSeen = now;
        }
      }
      std::erase_if(tracks_, [&](const auto &track)
                    {
      const auto local = (track.position - pose.Translation()).RotateBy(-pose.Rotation());
      return now - track.lastSeen > offenseconfig::kTrackRetentionSeconds || local.X().value() < offenseconfig::kTrackMinX || local.X().value() > offenseconfig::kTrackMaxX ||
             std::abs(local.Y().value()) > offenseconfig::kTrackMaxAbsY; });

  }

void OffensiveFuelEstimate::StartCollection()
  { tracks_.clear();
  }

void OffensiveFuelEstimate::CompleteFeed(bool fed)
  {
      if (fed)
        count_ = 0;

  }

void OffensiveFuelEstimate::AccountPartialFeed(double elapsed, double fullDuration, double capacity)
  {
      if (!std::isfinite(elapsed) || elapsed <= 0 || !std::isfinite(fullDuration) || fullDuration <= 0 ||
          !std::isfinite(capacity) || capacity <= 0)
        return;
      const double fed = std::floor(std::min(elapsed / fullDuration, 1.) * capacity);
      count_ = static_cast<int>(std::max(0., static_cast<double>(count_) - fed));

  }

int OffensiveFuelEstimate::Count() const
  { return count_;
  }

void OffensivePickupSelector::Reset()
  { *this = {};
  }

std::optional<frc::Translation2d> OffensivePickupSelector::Target() const
  { return target_;
  }

std::optional<frc::Pose2d> OffensivePickupSelector::Update(const std::vector<base::ObjectCluster> &points, frc::Pose2d pose,
                                      frc::ChassisSpeeds fieldSpeeds, double now, const FieldGeometry &field,
                                      bool prioritizeNearby,
                                      const std::function<bool(frc::Translation2d)> &passable)
  {
      // Keep the through-point on the pickup corridor, but shorten it when the
      // static planner grid blocks the original endpoint. The intake projects
      // forward, so a point slightly before the observation can still collect.
      const auto endpoint = [&](frc::Translation2d target, frc::Rotation2d direction)
          -> std::optional<frc::Translation2d>
      {
        for (int step = offenseconfig::kPickupSearchForwardSteps; step >= offenseconfig::kPickupSearchBackwardSteps; --step)
        {
          const auto through = target + frc::Translation2d{units::meter_t{step * offenseconfig::kPickupSearchStepMetres}, direction};
          const frc::Translation2d clipped{units::meter_t{std::clamp(through.X().value(), offenseconfig::kFieldEdgeInsetMetres, field.length - offenseconfig::kFieldEdgeInsetMetres)},
                                           units::meter_t{std::clamp(through.Y().value(), offenseconfig::kFieldEdgeInsetMetres, field.width - offenseconfig::kFieldEdgeInsetMetres)}};
          if (!passable || passable(clipped))
            return clipped;
        }
        return std::nullopt;
      };
      if (target_)
      {
        const auto relative = (pose.Translation() - *target_).RotateBy(-direction_);
        const double distance = pose.Translation().Distance(*target_).value();
        if (distance < bestDistance_ - offenseconfig::kPickupProgressMetres)
        {
          bestDistance_ = distance;
          progress_ = now;
        }
        const bool passed = relative.X().value() >= offenseconfig::kPickupPassedX && std::abs(relative.Y().value()) < offenseconfig::kPickupPassedMaxY &&
                            relative.X().value() - startProjection_ >= offenseconfig::kPickupProgressMetres;
        // A through-point clipped to the carpet edge cannot be driven past.
        // Once reached, select another pickup rather than waiting for the
        // ordinary lack-of-progress timeout at the wall.
        const auto through = *target_ + frc::Translation2d{units::meter_t{offenseconfig::kPickupThroughMetres}, direction_};
        const auto end = endpoint(*target_, direction_);
        const bool clipped = end && end->Distance(through).value() > 1e-6;
        const bool reachedEdge = clipped && pose.Translation().Distance(*end).value() < offenseconfig::kPickupEdgeArrivalMetres;
        const bool stuck = now - progress_ > offenseconfig::kPickupStuckSeconds;
        if (!end || passed || stuck || reachedEdge)
        {
          excluded_ = target_;
          excludeUntil_ = now + (stuck ? offenseconfig::kPickupStuckExcludeSeconds : offenseconfig::kPickupExcludeSeconds);
          target_.reset();
        }
      }
      if (!target_)
      {
        // Rank from where the intake is headed over a short horizon, while
        // preserving the committed corridor until the pickup is passed.
        const auto intake = pose.Translation() + frc::Translation2d{units::meter_t{offenseconfig::kIntakeForwardMetres}, pose.Rotation()} +
                            frc::Translation2d{units::meter_t{fieldSpeeds.vx.value() * offenseconfig::kPickupPredictionSeconds},
                                               units::meter_t{fieldSpeeds.vy.value() * offenseconfig::kPickupPredictionSeconds}};
        const base::ObjectCluster *best = nullptr;
        double bestCost = std::numeric_limits<double>::infinity();
        std::vector<const base::ObjectCluster *> candidates;
        candidates.reserve(points.size());
        for (const auto &point : points)
          if ((!excluded_ || now >= excludeUntil_ || point.position.Distance(*excluded_).value() >= offenseconfig::kPickupMatchDistance) &&
              endpoint(point.position, (point.position - pose.Translation()).Angle()))
            candidates.push_back(&point);
        const auto count = std::min<std::size_t>(offenseconfig::kPickupCandidateCount, candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end(),
                          [&](const auto *a, const auto *b)
                          { return a->position.Distance(prioritizeNearby ? pose.Translation() : intake) <
                                   b->position.Distance(prioritizeNearby ? pose.Translation() : intake); });
        for (std::size_t i = 0; i < count; ++i)
        {
          const auto &candidate = *candidates[i];
          const auto delta = candidate.position - intake;
          const double distance = delta.Norm().value();
          const double angle = std::abs(std::remainder((delta.Angle() - pose.Rotation()).Radians().value(),
                                                       2 * std::numbers::pi));
          const double projected = distance > 1e-6
                                       ? (fieldSpeeds.vx.value() * delta.X().value() +
                                          fieldSpeeds.vy.value() * delta.Y().value()) /
                                             distance
                                       : 0.;
          double yield = 0.;
          for (const auto &other : points)
            if (candidate.position.Distance(other.position).value() <= offenseconfig::kPickupYieldRadius)
              yield += std::clamp(other.confidence, 0., 1.);
          // Penalize turning and reversing to favor pickups along the current path.
          const double age = std::clamp(now - candidate.captureTime.value(), 0., offenseconfig::kPickupMaxAgeSeconds);
          const double cost = prioritizeNearby ? candidate.position.Distance(pose.Translation()).value() : (distance + offenseconfig::kPickupTurnWeight * angle + offenseconfig::kPickupReverseWeight * std::max(0., -projected) + offenseconfig::kPickupAgeWeight * age) / std::max(offenseconfig::kPickupMinYield, yield);
          if (cost < bestCost)
          {
            bestCost = cost;
            best = &candidate;
          }
        }
        if (!best)
          return std::nullopt;
        target_ = best->position;
        direction_ = (*target_ - pose.Translation()).Angle();
        startProjection_ = (pose.Translation() - *target_).RotateBy(-direction_).X().value();
        progress_ = now;
        bestDistance_ = pose.Translation().Distance(*target_).value();
      }
      const auto end = endpoint(*target_, direction_);
      return end ? std::optional{frc::Pose2d{*end, direction_}} : std::nullopt;

  }
} // namespace season
