#pragma once
#ifndef __FRC_ROBORIO__
#include "base/sim/RobotContactConfig.h"
#endif


namespace season::fuelconfig
{
    // Desktop fuel-model configuration. These values also define the nominal
    // rate used by offensive feed timing when no simulation preference is set.
    inline constexpr double kFuelDiameterMetres = 5.5 * .0254;
    inline constexpr double kFieldReturnInsetMetres = .30;
    inline constexpr double kBumperHalfWidthMetres = .43, kBumperDepthMetres = .0889;
    inline constexpr double kBumperBottomMetres = .0127, kBumperHalfHeightMetres = .06985;
    inline constexpr double kMinHoodDeflectionDegrees = 15, kMaxHoodDeflectionDegrees = 45;
    inline constexpr double kHopperFloorMetres = .18, kHopperCeilingMetres = .65;
    inline constexpr double kHopperBackMetres = -.33, kHopperHalfWidthMetres = .34;
    inline constexpr double kBurstLateralCenter = 1.5, kBurstLateralSpacingMetres = .08;
    inline constexpr double kMinLaunchPitchDegrees = 45, kMaxLaunchPitchDegrees = 75;
    inline constexpr double kLaunchPitchNoiseDegrees = 1, kReturnAngleNoiseDegrees = 45, kLaunchSpeedNoiseFraction = .01;
    inline constexpr double kBurstResetPeriods = 2;
    inline constexpr double kMaxStepSeconds = .25, kScoringGraceSeconds = 3.;
    inline constexpr double kCollectionMinRackMetres = .12, kCollectionMouthXMetres = .28;
    inline constexpr double kCollectionMouthDepthMetres = .20, kCollectionHalfWidthMetres = .35;
    inline constexpr double kMinFeedFlywheelRps = 5, kMinLaunchGapSeconds = .01;
    inline constexpr double kLaunchGapSeconds = .05, kLaunchGapNoiseSeconds = .015, kParticleStepSeconds = .005;
    inline constexpr double kHubHalfWidthMetres = .6096, kScoredReturnOffsetMetres = .65;
    inline constexpr double kScoredReturnSpeedMps = 3, kGroundBounceScale = .15, kFloorDampingPerSecond = 2;
    inline constexpr double kFieldRestitution = .25, kRobotParticleFriction = .30;
    inline constexpr double kStartingFuelMinX = 7.0, kStartingFuelMaxX = 9.5, kStartingFuelSpacing = .25;
    inline constexpr double kStartingFuelMinY = .75, kStartingFuelMaxY = 7.25;
    inline constexpr double kShooterLoadDurationSeconds = .08, kLoadNoiseMin = .35, kLoadNoiseMax = 1.65;
    inline constexpr double kTransportBaseLoad = .25, kTransportFillLoad = .75;
    inline constexpr double kConveyorLoadTorque = .16, kIndexerLoadTorque = .12, kShooterLoadTorque = .35;
    inline constexpr int kSimulationCapacity = 54;
    inline constexpr int kSimulationBurstSize = 4;
    inline constexpr double kSimulationBurstPeriodSeconds = .20;
    inline constexpr double kSimulationFeedBallsPerSecond =
        kSimulationBurstSize / kSimulationBurstPeriodSeconds;
}

namespace season::simulation::motorconfig
{
  inline constexpr double kNominalVoltage = 12.;
  inline constexpr double kNeoStallCurrentAmps = 105, kNeoStallTorqueNm = 2.6, kNeoFreeSpeedRpm = 5676;
  inline constexpr double kKrakenStallCurrentAmps = 366, kKrakenStallTorqueNm = 7.09, kKrakenFreeSpeedRpm = 6000;
  inline constexpr double kMaxUpdateSeconds = 1;
  inline constexpr double kMaxPhysicsStepSeconds = .0002, kLoadDirectionSpeedThreshold = .01;
  inline constexpr double kViscousDrag = .0001;
}

namespace season::simulation::runtimeconfig
{
  inline constexpr int kResetFeedbackLoops = 5, kQuestAlignmentMinTags = 2;
  inline constexpr unsigned kFuelRandomSeed = 3737;
  inline constexpr double kSimulationPeriodSeconds = .02;
  inline constexpr double kLimelightHorizontalFovDegrees = 62.5, kLimelightVerticalFovDegrees = 48.9;
  inline constexpr double kQuestHorizontalFovDegrees = 90, kQuestVerticalFovDegrees = 90, kVisionRangeMetres = 6;
  inline constexpr double kVisionTagDistanceNoiseMetres = .75;
  inline constexpr double kLimelightStdX = .08, kLimelightStdY = .08, kLimelightStdHeading = .12;
  inline constexpr double kQuestStdX = .1, kQuestStdY = .1, kQuestStdHeading = .05;
  inline constexpr double kRampClearancePaddingMetres = .005, kSpawnLaneWidthInches = 50.34;
  inline constexpr double kSpawnXMetres = 3.65, kFootprintHalfLengthMetres = .43, kFootprintHalfWidthMetres = .43;
  inline constexpr double kFootprintHeightMetres = .45, kSpawnClearanceMetres = .02;
  inline constexpr int kPracticeAutonomousSeconds = 20, kPracticeAssessmentSeconds = 3, kPracticeTeleopSeconds = 140;
  inline constexpr double kBatteryCapacityAh = 18.0, kBatteryEmptyVoltage = 10.5;
  inline constexpr double kBatteryVoltageRange = 2.1, kBatteryInternalResistanceOhms = .020;
}

#ifndef __FRC_ROBORIO__
namespace season::simulation
{
  inline base::sim::RobotContactConfig RobotContacts()
  {
    constexpr double inchesToMetres = .0254;
    base::sim::RobotContactConfig c;
    // Unmeasured construction/material estimates: replace with loaded measurements.
    c.chassisClearance = 2.0 * inchesToMetres;
    c.panThickness = .125 * inchesToMetres;
    c.panHalfLength = c.panHalfWidth = .43 - 3.5 * inchesToMetres;
    c.bumperFloorGap = .5 * inchesToMetres; // User-confirmed flush foam/plywood floor gap.
    c.noodleDiameter = 2.75 * inchesToMetres;
    c.backingThickness = .75 * inchesToMetres;
    c.backingHeight = 5.0 * inchesToMetres;
    c.backingFloorGap = c.bumperFloorGap; // Selected flush plywood bottom is rigid.
    c.compressionStrain = {0, .25, .5, .7, .8};
    c.compressionStressPa = {0, 10000, 35000, 100000, 250000};
    // Overdamped foam dissipates more impact energy and reduces rebound.
    c.dampingRatio = 1.2;
    return c;
  }
}
#endif
