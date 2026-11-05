#pragma once
#include "season/drive/AutoDrive.h"
#include "season/drive/ScoringPosition.h"
#include <pathplanner/lib/path/PathPlannerPath.h>
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
  // Fuel observation and estimated possession.
  // Forward and lateral observation bounds for fuel eligible for pickup tracking, in metres.
  inline constexpr double kObserveMinX = .15, kObserveMaxX = 1.6, kObserveMaxAbsY = .65;
  // Maximum displacement for associating a new observation with an existing fuel track, in metres.
  inline constexpr double kTrackMatchDistance = .12;
  // Intake mouth bounds used to detect fuel entry relative to the robot, in metres.
  inline constexpr double kMouthMinX = .20, kMouthMaxX = .70, kMouthHalfWidth = .4;
  // Time an unseen fuel track is retained to bridge brief occlusions, in seconds.
  inline constexpr double kTrackRetentionSeconds = 2.;
  // Rear, forward, and lateral bounds for retaining fuel tracks relative to the robot, in metres.
  inline constexpr double kTrackMinX = -.7, kTrackMaxX = 2.5, kTrackMaxAbsY = 1.5;

  // Ferry timing.
  // Time until the next HUB state change at which the ferry window is closing, in seconds.
  inline constexpr double kFerryClosingSeconds = 15.;

  // Pickup selection and tracking.
  // Maximum distance for matching an observed pickup with the committed target, in metres.
  inline constexpr double kPickupMatchDistance = .3;
  // Inclusive maximum and minimum signed step indices for pickup endpoint search.
  inline constexpr int kPickupSearchForwardSteps = 8, kPickupSearchBackwardSteps = -3;
  // Spacing between adjacent sampled pickup search offsets, in metres.
  inline constexpr double kPickupSearchStepMetres = .1;
  // Inset used to clamp candidate pickup endpoints from field boundaries, in metres.
  inline constexpr double kFieldEdgeInsetMetres = .5;
  // Required progress toward a committed pickup before its progress timer resets, in metres.
  inline constexpr double kPickupProgressMetres = .15;
  // Rearward X threshold indicating the robot has passed a pickup, in metres.
  inline constexpr double kPickupPassedX = -.35;
  // Maximum lateral offset for a pickup to count as passed, in metres.
  inline constexpr double kPickupPassedMaxY = .55;
  // Reference distance through a pickup used to identify a shortened endpoint, in metres.
  inline constexpr double kPickupThroughMetres = .8;
  // Distance at which a shortened pickup endpoint is treated as arrived, in metres.
  inline constexpr double kPickupEdgeArrivalMetres = .12;
  // No-progress and target exclusion durations for abandoning a stuck or rejected pickup, in seconds.
  inline constexpr double kPickupStuckSeconds = 3., kPickupStuckExcludeSeconds = 2., kPickupExcludeSeconds = .8;
  // Forward intake offset from the robot reference point, in metres.
  inline constexpr double kIntakeForwardMetres = .58;
  // Prediction horizon for the intake position using robot field-relative velocity, in seconds.
  inline constexpr double kPickupPredictionSeconds = .2;
  // Number of pickup candidates retained for scoring and selection.
  inline constexpr std::size_t kPickupCandidateCount = 4;
  // Radius around a candidate used to sum nearby observation confidence as estimated pickup yield, in metres.
  inline constexpr double kPickupYieldRadius = 1.22;
  // Maximum observation age used in the candidate cost penalty, in seconds; this does not reject older observations.
  inline constexpr double kPickupMaxAgeSeconds = 2.;
  // Candidate-selection cost weights for turning, reversing, and observation age.
  inline constexpr double kPickupTurnWeight = .65, kPickupReverseWeight = .5, kPickupAgeWeight = .4;
  // Lower bound on estimated yield in the candidate cost denominator.
  inline constexpr double kPickupMinYield = .5;

  // Return and field crossing strategy.
  // Blue field X coordinate of the home point for scoring-window estimates, in metres; mirrored for red.
  inline constexpr double kReturnHomeXMetres = 3.65;
  // Blue field X coordinate of the trench point for scoring-window estimates, in metres; mirrored for red.
  inline constexpr double kReturnTrenchXMetres = 6.2;
  // Setup-time allowance added to route distance treated as seconds (an implicit 1 m/s estimate).
  inline constexpr double kReturnSetupSeconds = 12.;
  // Autonomous match time threshold for beginning the return sequence, in seconds.
  inline constexpr double kAutonomousReturnSeconds = 13.;
  // Lateral offset used to route through a trench lane, in metres.
  inline constexpr double kTrenchLaneOffset = .75;
  // Field X offset from the HUB used to stage a crossing, in metres.
  inline constexpr double kHubCrossingOffset = 1.8;

  // Collection route selection.
  // Minimum and maximum spacing between collection route candidates, in metres.
  inline constexpr double kCollectionMinSeparation = .35, kCollectionMaxSeparation = 3.;
  // Largest turn after the first pickup segment when evaluating a collection route, in radians.
  inline constexpr double kCollectionMaxTurnRadians = 2.1;
  // Turn cost weight while the hub is active and scoring is preferred.
  inline constexpr double kActiveTurnWeight = .1;
  // Turn cost weight while the hub is inactive and collection is preferred.
  inline constexpr double kInactiveTurnWeight = .5;
  // Maximum pickup points selected for a collection spline.
  inline constexpr int kMaxCollectionCandidates = 12;

  // Match timing and empty-load estimation.
  // Fallback speed and clamp bounds for the planner's travel-speed estimate, in metres per second.
  inline constexpr double kFallbackTravelSpeed = .75, kMinEstimatedTravelSpeed = .6, kMaxEstimatedTravelSpeed = 1.;
  // Scheduling buffer for preparing to score before the HUB becomes active, in seconds.
  inline constexpr double kHubPreparationMarginSeconds = 1.5;
  // Smoothing gain for the roller current means used to estimate current fluctuations.
  inline constexpr double kEmptyCurrentFilterGain = .2;
  // Quiet duration before current indicates an empty load, in seconds.
  inline constexpr double kEmptyQuietSeconds = .4;

  // Navigation tracking and command time bounds.
  // Offset from the hub used to define the crossing-zone route, in metres.
  inline constexpr double kCrossingZoneOffset = .8;
  // Distance tolerance for reaching a hub-crossing waypoint, in metres.
  inline constexpr double kCrossingWaypointTolerance = .4;
  // Minimum stable tracking duration before acting on a target, in seconds.
  inline constexpr double kTrackingStabilitySeconds = .05;
  // Maximum linear and angular speeds for considering target tracking settled, in metres and radians per second.
  inline constexpr double kTrackingLinearSettleSpeed = .4, kTrackingAngularSettleSpeed = .5;
  // Minimum collection time before estimating refill duration from inferred pickups, in seconds.
  inline constexpr double kCollectionSpeedSampleSeconds = 1.;
  // Maximum run times for the collection, retreat, and entry commands, in seconds.
  inline constexpr units::second_t kCollectionTimeout{20}, kRetreatTimeout{3}, kEntryTimeout{8};
  // Maximum run times for the scoring approach or ferry bump clearance, ferry entry, and ferry approach, in seconds.
  inline constexpr units::second_t kScoringApproachTimeout{5}, kFerryEntryTimeout{7}, kFerryApproachTimeout{9};
  // Overall bounds for returning through the trench or directly to a scoring position.
  inline constexpr units::second_t kTrenchReturnTimeout{18}, kDirectReturnTimeout{15};
  // Bound for driving directly to a ferry position, including after clearing a bump.
  inline constexpr units::second_t kFerryRepositionTimeout{10};
  // Pause after a staging return before repeating the stage cycle, in seconds.
  inline constexpr units::second_t kReturnDelay{.1};
  // Delay between evaluating offense decisions, in seconds.
  inline constexpr units::second_t kDecisionDelay{.02};
  // Speed used to clear the depot before planning a return, in metres per second.
  inline constexpr double kClearSpeedMps = 2.;
  // Speed used while clearing a bump before planning a return, in metres per second.
  inline constexpr double kRetreatSpeedMps = .8;
  // Clearance buffer used when retreating from a bump or depot, in metres.
  inline constexpr double kRetreatBufferMetres = .15;
  // Entry completion and approach handoff position tolerances, in metres.
  inline constexpr double kEntryToleranceMetres = .35, kEntryHandoffToleranceMetres = .5;
  // Position and heading differences from the scoring goal below which navigation is skipped, in metres and radians.
  inline constexpr double kGoalPositionEqualityMetres = .01, kGoalHeadingEqualityRadians = .04;
  // Position tolerance for scoring-goal proximity checks, in metres.
  inline constexpr double kGoalArrivalMetres = .15;
  // Position tolerance from the ferry goal below which repositioning is skipped, in metres.
  inline constexpr double kFerryGoalEqualityMetres = .05;
  // Maximum translation speed used during a ferry alignment, in metres per second.
  inline constexpr double kFerryTranslationSpeedMps = .2;
  // Elapsed collection time after which the next decision treats the attempt as expired, in seconds.
  inline constexpr double kCollectionDecisionSeconds = 19.9;
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

namespace season::offense
{
  // Drivetrain limits and collection routes used by offensive autonomous.
  base::AutoDriveConfig OffensiveMotionLimits(const base::DriveConfig &drive);
  std::shared_ptr<pathplanner::PathPlannerPath> CollectionSpline(
      frc::Pose2d start, const std::vector<base::ObjectCluster> &observations,
      frc::DriverStation::Alliance alliance, const FieldGeometry &field, const ScoringGrid &grid,
      bool active, bool ferry, int remaining, const base::AutoDriveConfig &limits);
}
