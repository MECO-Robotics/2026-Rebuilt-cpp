#pragma once
#include "season/drive/AutoDrive.h"
#include "season/mechanisms/Mechanisms.h"
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace season
{
  // A timeout or unknown/inactive HUB must never authorize a hub shot.
  bool OffensiveHubShotAllowed(std::optional<bool> hubActive, std::optional<frc::Pose2d> target, frc::Pose2d pose,
                                      frc::ChassisSpeeds speeds);
  // Anonymous detections can disappear due to pickup or occlusion. This is a
  // heuristic, not a measured hopper count.
  class OffensiveFuelEstimate
  {
  public:
    void Observe(const std::vector<base::ObjectCluster> &points, frc::Pose2d pose);
    void StartCollection();
    void CompleteFeed(bool fed);
    void AccountPartialFeed(double elapsed, double fullDuration, double capacity);
    int Count() const;

  private:
    struct Track
    {
      frc::Translation2d position;
      double lastSeen;
      bool seen, consumed, enteredMouth;
    };
    int count_ = 0;
    std::vector<Track> tracks_;
  };
  enum class OffensiveDecision
  {
    Wait,
    Collect,
    Score,
    Ferry,
    Stage
  };
  enum class OffensiveCommitment
  {
    None,
    Score,
    Ferry
  };
  bool OffensiveFerryWindowClosing(std::optional<double> nextChange);
  std::optional<frc::Translation2d> MatchOffensiveTarget(frc::Translation2d locked,
                                                                const std::vector<base::ObjectCluster> &points);
  // A committed corridor prevents small observation changes from rotating the
  // approach on every scheduler tick. Keep the last observed target through
  // occlusion; passing it or failing to make progress releases the lock.
  class OffensivePickupSelector
  {
  public:
    void Reset();
    std::optional<frc::Translation2d> Target() const;
    std::optional<frc::Pose2d> Update(const std::vector<base::ObjectCluster> &points, frc::Pose2d pose,
                                      frc::ChassisSpeeds fieldSpeeds, double now, const FieldGeometry &field,
                                      bool prioritizeNearby = false,
                                      const std::function<bool(frc::Translation2d)> &passable = {});

  private:
    std::optional<frc::Translation2d> target_, excluded_;
    frc::Rotation2d direction_{};
    double progress_ = 0., bestDistance_ = 0., excludeUntil_ = 0., startProjection_ = 0.;
  };
  OffensiveCommitment ReconcileOffensiveCommitment(OffensiveCommitment current, std::optional<bool> hubActive,
                                                          bool fullFeedCompleted);
  // Estimate enough time for the staged trench route and shooter readiness.
  // This is a conservative scheduling heuristic, not an arrival guarantee.
  bool OffensiveScoringWindowClosing(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                                            const FieldGeometry &field, std::optional<double> nextChange);
  // Upstream single-robot arbitration adapted to an estimated, not measured, load.
  OffensiveDecision DecideOffense(std::optional<bool> active, std::optional<frc::DriverStation::Alliance> alliance,
                                         frc::Pose2d pose, const FieldGeometry &field,
                                         const std::vector<base::ObjectCluster> &points, int estimatedFuel,
                                         double capacity, bool scoreCommitted = false, bool ferryCommitted = false,
                                         bool autonomous = false, std::optional<double> periodTime = std::nullopt,
                                         bool collectionExpired = false, bool loadUnknown = true,
                                         std::optional<frc::Translation2d> retainedPickup = std::nullopt);
  // Runs until interrupted. Possession is not sensed on this robot; collection
  // and shooting are bounded attempts rather than exact capacity-based decisions.
  frc2::CommandPtr OffensiveAutoDrive(AutoDrive &perception, base::Drive &drive, Mechanisms mechanisms,
                                      FieldGeometry field, std::shared_ptr<OffensiveFuelEstimate> estimate = {});
} // namespace season
