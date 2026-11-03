#pragma once
#include "season/drive/AutoDrive.h"
#include "season/mechanisms/Mechanisms.h"
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include <cstddef>
#include <units/time.h>

namespace season
{
namespace offenseconfig
{
  // Fuel observation and pickup tracking, in metres and seconds.
  inline constexpr double kObserveMinX = .15, kObserveMaxX = 1.6, kObserveMaxAbsY = .65;
  inline constexpr double kTrackMatchDistance = .12, kMouthMinX = .20, kMouthMaxX = .70, kMouthHalfWidth = .4;
  inline constexpr double kTrackRetentionSeconds = 2., kTrackMinX = -.7, kTrackMaxX = 2.5, kTrackMaxAbsY = 1.5;
  inline constexpr double kFerryClosingSeconds = 15., kPickupMatchDistance = .3;
  inline constexpr int kPickupSearchForwardSteps = 8, kPickupSearchBackwardSteps = -3;
  inline constexpr double kPickupSearchStepMetres = .1, kFieldEdgeInsetMetres = .5;
  inline constexpr double kPickupProgressMetres = .15, kPickupPassedX = -.35, kPickupPassedMaxY = .55;
  inline constexpr double kPickupThroughMetres = .8, kPickupEdgeArrivalMetres = .12;
  inline constexpr double kPickupStuckSeconds = 3., kPickupStuckExcludeSeconds = 2., kPickupExcludeSeconds = .8;
  inline constexpr double kIntakeForwardMetres = .58, kPickupPredictionSeconds = .2;
  inline constexpr std::size_t kPickupCandidateCount = 4;
  inline constexpr double kPickupYieldRadius = 1.22, kPickupMaxAgeSeconds = 2.;
  inline constexpr double kPickupTurnWeight = .65, kPickupReverseWeight = .5, kPickupAgeWeight = .4, kPickupMinYield = .5;
  inline constexpr double kReturnHomeXMetres = 3.65, kReturnTrenchXMetres = 6.2;
  inline constexpr double kReturnSetupSeconds = 12., kAutonomousReturnSeconds = 13.;
  // Corridor geometry and collection planner policy; distances are metres.
  inline constexpr double kTrenchLaneOffset = .75, kHubCrossingOffset = 1.8;
  inline constexpr double kCollectionMinSeparation = .35, kCollectionMaxSeparation = 3.;
  inline constexpr double kCollectionMaxTurnRadians = 2.1;
  inline constexpr double kActiveTurnWeight = .1, kInactiveTurnWeight = .5;
  inline constexpr int kMaxCollectionCandidates = 12;
  inline constexpr double kFallbackTravelSpeed = .75, kMinEstimatedTravelSpeed = .6, kMaxEstimatedTravelSpeed = 1.;
  inline constexpr double kHubPreparationMarginSeconds = 1.5;
  inline constexpr double kEmptyCurrentFilterGain = .2, kEmptyQuietSeconds = .4;
  inline constexpr double kSimEmptyShooterCurrentAmps = 2., kSimEmptyRollerNoiseAmps = .5;
  inline constexpr double kCrossingZoneOffset = .8, kCrossingWaypointTolerance = .4;
  inline constexpr double kTrackingStabilitySeconds = .05, kTrackingLinearSettleSpeed = .4, kTrackingAngularSettleSpeed = .5;
  inline constexpr double kCollectionSpeedSampleSeconds = 1.;
  inline constexpr units::second_t kCollectionTimeout{20}, kRetreatTimeout{3}, kEntryTimeout{8};
  inline constexpr units::second_t kScoringApproachTimeout{5}, kFerryEntryTimeout{7}, kFerryApproachTimeout{9};
  inline constexpr units::second_t kReturnDelay{.1}, kDecisionDelay{.02};
  inline constexpr double kClearSpeedMps = 2., kRetreatSpeedMps = .8, kRetreatBufferMetres = .15;
  inline constexpr double kEntryToleranceMetres = .35, kEntryHandoffToleranceMetres = .5;
  inline constexpr double kGoalPositionEqualityMetres = .01, kGoalHeadingEqualityRadians = .04;
  inline constexpr double kGoalArrivalMetres = .15, kFerryGoalEqualityMetres = .05;
  inline constexpr double kFerryTranslationSpeedMps = .2, kCollectionDecisionSeconds = 19.9;
}

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
