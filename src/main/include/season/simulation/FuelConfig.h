#pragma once

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
