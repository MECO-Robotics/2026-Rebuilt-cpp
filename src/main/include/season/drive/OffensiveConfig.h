#pragma once
#include <units/time.h>
#include <cstddef>
namespace season::offenseconfig
{
  inline constexpr double kArrivalDistanceMetres = .05, kArrivalHeadingRadians = .06;
  inline constexpr double kArrivalLinearSpeed = .2, kArrivalAngularSpeed = .2, kDetectionPeriodSeconds = .1;
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
