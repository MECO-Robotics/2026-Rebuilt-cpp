#pragma once

#include "season/shooting/Shooting.h"

// Mechanism positions and voltages used by commands.
namespace season::mechanismconfig
{
  // Conveyor voltage used by autonomous intake acquisition.
  inline constexpr double kAutoIntakeConveyorVolts = 5;
  // Conveyor voltage applied during command-based acquisition.
  inline constexpr double kAcquireConveyorVolts = 5;
  // Bottom indexer voltage applied to pull game pieces inward during acquisition.
  inline constexpr double kAcquireBottomVolts = 5;
  // Top indexer voltage applied to pull game pieces inward during acquisition.
  inline constexpr double kAcquireTopVolts = -5;
  // Conveyor voltage applied while ejecting game pieces.
  inline constexpr double kEjectConveyorVolts = -5;
  // Bottom indexer voltage applied while ejecting game pieces.
  inline constexpr double kEjectBottomVolts = 10;
  // Top indexer voltage applied while ejecting game pieces.
  inline constexpr double kEjectTopVolts = -10;
  // Conveyor voltage used to stow a piece through the mechanism.
  inline constexpr double kStowConveyorVolts = 10;
  // Rack encoder positions in metres for stowed, deployed, and clearance-safe states, respectively.
  inline constexpr double kRackStow = 0, kRackDeploy = .30, kRackSafe = .12;
  // Intake voltage used for normal game-piece intake.
  inline constexpr double kIntakeVolts = -10;
  // Intake voltage used for slower controlled movement.
  inline constexpr double kIntakeSlowVolts = 7;
  // Intake voltage used to eject a game piece.
  inline constexpr double kEjectVolts = 10;
  // Conveyor voltage used to feed a game piece toward the shooter.
  inline constexpr double kConveyorFeedVolts = -11;
  // Bottom and top indexer voltages used during shooter feeding, respectively.
  inline constexpr double kBottomFeedVolts = -10, kTopFeedVolts = 10;
}

// Shooting presets, readiness limits, timing, and feedback.
namespace season::shotconfig
{
  // Rack retraction speed in m/s while a shot is being prepared.
  inline constexpr double kShotRetractSpeedMps = .10;
  // Rack extension held during the shot sequence, in metres.
  inline constexpr double kShotHoldPositionMetres = .13;
  // Intake acceleration limit while running the shot agitation sequence.
  inline constexpr double kShotIntakeAcceleration = 100;
  // Maximum wait for the robot to become ready for a hub shot.
  inline constexpr double kHubReadyTimeoutSeconds = 5;
  // Maximum translational speed in m/s allowed for a moving-ferry shot.
  inline constexpr double kMovingFerryMaxSpeed = 1.;
  // Maximum translational speed in m/s allowed for a stationary shot.
  inline constexpr double kStationaryShotMaxSpeed = .25;
  // Maximum angular speed in rad/s allowed for a moving-ferry shot.
  inline constexpr double kMovingFerryMaxAngularSpeed = .3;
  // Maximum angular speed in rad/s allowed before a stationary shot proceeds.
  inline constexpr double kStationaryShotMaxAngularSpeed = 10 * season::kPi / 180;
  // Shooter velocity error in RPS tolerated when evaluating shot readiness.
  inline constexpr double kShotVelocityToleranceRps = 3;
  // Controller rumble intensity while game pieces are actively feeding.
  inline constexpr double kRumbleFeedIntensity = .35;
  // Controller rumble intensity used for each readiness pulse.
  inline constexpr double kRumblePulseIntensity = .75;
  // Period in seconds for repeating shot-readiness rumble pulses.
  inline constexpr double kRumblePeriodSeconds = 1.;
  // End time in seconds of the first readiness rumble pulse in each period.
  inline constexpr double kRumbleFirstPulseEndSeconds = .15;
  // Start time in seconds of the second readiness rumble pulse in each period.
  inline constexpr double kRumbleSecondPulseStartSeconds = .30;
  // End time in seconds of the second readiness rumble pulse in each period.
  inline constexpr double kRumbleSecondPulseEndSeconds = .45;
  // Maximum time in seconds for a shot to reach its ready state.
  inline constexpr double kShotReadyTimeoutSeconds = 8;
  // Maximum total shot-command durations in seconds for desktop simulation and the real robot, respectively.
  inline constexpr double kShotSimTimeoutSeconds = 25, kShotRealTimeoutSeconds = 15;
  // Hood output rotations for the hub, ferry, and trench shooting presets, respectively.
  inline constexpr double kHubHood = 0, kFerryHood = .049, kTrenchHood = .049;
  // Shooter target speeds in RPS for the hub, ferry, and trench presets, respectively.
  inline constexpr double kHubRps = 30, kFerryRps = 55, kTrenchRps = 35;
  // Autonomous shot timing in seconds: readiness wait, fallback feed duration, and total timeout, respectively.
  inline constexpr double kAutoReadyTimeoutSeconds = 2, kAutoFeedSeconds = 1, kAutoTotalTimeoutSeconds = 3;
}

// Driver and autonomous aiming settings.
namespace season::driveconfig
{
  // Joystick magnitude below this fraction is treated as zero by driver controls.
  inline constexpr double kDriverDeadband = .1;
  // Heading-error proportional gain used by manual aiming control.
  inline constexpr double kAimHeadingGain = 5;
  // Heading-error proportional gain used by autonomous aiming control.
  inline constexpr double kAutoAimHeadingGain = 7;
  // Maximum angular rate in rad/s commanded by autonomous aiming.
  inline constexpr double kAutoAimMaxAngularRate = 7;
}

// Vision observation freshness.
namespace season::visionconfig
{
  // Maximum age in seconds for a vision observation to be considered fresh.
  inline constexpr double kVisionFreshnessSeconds = .5;
}

// Transport feedback assumptions for desktop simulation.
namespace season::simulationconfig
{
  // Minimum voltage magnitude used by simulation to consider a transport motor commanded.
  inline constexpr double kSimTransportVoltageThreshold = 1;
}
