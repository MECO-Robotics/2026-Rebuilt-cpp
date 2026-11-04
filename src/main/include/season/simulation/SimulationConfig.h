#pragma once
#include "season/RobotGeometry.h"
#ifndef __FRC_ROBORIO__
#include "base/sim/RobotContactConfig.h"
#endif


namespace season::simulationconfig
{
  // Minimum voltage magnitude used by simulation to consider a transport motor commanded.
  inline constexpr double kSimTransportVoltageThreshold = 1;
}

namespace season::fuelconfig
{
    // Diameter of each simulated fuel particle, converted from the game-piece inch measurement.
    inline constexpr double kFuelDiameterMetres = 5.5 * .0254;
    // Inset from the field boundary where returned fuel is placed.
    inline constexpr double kFieldReturnInsetMetres = .30;
    // Bumper collision volume dimensions relative to the robot and field floor.
    inline constexpr double kBumperHalfWidthMetres = geometry::kWidthMetres / 2, kBumperDepthMetres = .0889,
                            kBumperBottomMetres = .0127, kBumperHalfHeightMetres = .06985;
    // Bounds on hood deflection accepted by the simulated shooter.
    inline constexpr double kMinHoodDeflectionDegrees = 15, kMaxHoodDeflectionDegrees = 45;
    // Hopper interior boundaries relative to the robot reference frame.
    inline constexpr double kHopperFloorMetres = .18, kHopperCeilingMetres = .65,
                            kHopperBackMetres = -.33, kHopperHalfWidthMetres = .34;
    // Lateral placement of particles in the burst pattern.
    inline constexpr double kBurstLateralCenter = 1.5, kBurstLateralSpacingMetres = .08;
    // Bounds on launch pitch used by the shooter model.
    inline constexpr double kMinLaunchPitchDegrees = 45, kMaxLaunchPitchDegrees = 75;
    // Standard deviation applied to simulated launch pitch.
    inline constexpr double kLaunchPitchNoiseDegrees = 1;
    // Maximum random angular deviation applied when scored fuel returns to the field.
    inline constexpr double kReturnAngleNoiseDegrees = 45;
    // Fractional standard deviation applied to simulated launch speed.
    inline constexpr double kLaunchSpeedNoiseFraction = .01;
    // Number of simulation update periods a burst reset takes.
    inline constexpr double kBurstResetPeriods = 2;
    // Upper bound on the elapsed time one fuel-model update may integrate.
    inline constexpr double kMaxStepSeconds = .25;
    // Time after a scoring event during which the score remains eligible for simulation handling.
    inline constexpr double kScoringGraceSeconds = 3.;
    // Collection contact geometry relative to the robot center.
    inline constexpr double kCollectionMinRackMetres = .12, kCollectionMouthXMetres = .28,
                            kCollectionMouthDepthMetres = .20, kCollectionHalfWidthMetres = .35;
    // Minimum flywheel speed for the model to accept a fuel feed.
    inline constexpr double kMinFeedFlywheelRps = 5;
    // Minimum simulated time between consecutive launches.
    inline constexpr double kMinLaunchGapSeconds = .01;
    // Nominal time between consecutive fuel launches.
    inline constexpr double kLaunchGapSeconds = .05;
    // Random variation applied to the nominal launch spacing.
    inline constexpr double kLaunchGapNoiseSeconds = .015;
    // Integration interval for advancing individual fuel particles.
    inline constexpr double kParticleStepSeconds = .005;
    // Half-width of the scoring hub region used by the fuel model.
    inline constexpr double kHubHalfWidthMetres = .6096;
    // Distance from the hub at which scored fuel is reintroduced.
    inline constexpr double kScoredReturnOffsetMetres = .65;
    // Initial speed assigned to fuel returned after scoring.
    inline constexpr double kScoredReturnSpeedMps = 3;
    // Multiplier applied to vertical velocity when fuel bounces from the floor.
    inline constexpr double kGroundBounceScale = .15;
    // Rate at which horizontal particle velocity is damped on the floor.
    inline constexpr double kFloorDampingPerSecond = 2;
    // Coefficient of restitution for fuel collisions with field surfaces.
    inline constexpr double kFieldRestitution = .25;
    // Friction coefficient for fuel particles contacting the robot.
    inline constexpr double kRobotParticleFriction = .30;
    // Initial fuel placement region bounds and spacing, in metres.
    inline constexpr double kStartingFuelMinX = 7.0, kStartingFuelMaxX = 9.5,
                            kStartingFuelSpacing = .25, kStartingFuelMinY = .75,
                            kStartingFuelMaxY = 7.25;
    // Duration of the modeled shooter loading interval.
    inline constexpr double kShooterLoadDurationSeconds = .08;
    // Bounds on the randomized multiplier applied to transport load.
    inline constexpr double kLoadNoiseMin = .35, kLoadNoiseMax = 1.65;
    // Baseline load contribution from the transport system.
    inline constexpr double kTransportBaseLoad = .25;
    // Additional transport load at full simulated fuel capacity.
    inline constexpr double kTransportFillLoad = .75;
    // Load torque applied to the simulated conveyor motor.
    inline constexpr double kConveyorLoadTorque = .16;
    // Load torque applied to the simulated indexer motor.
    inline constexpr double kIndexerLoadTorque = .12;
    // Load torque applied to the simulated shooter motor.
    inline constexpr double kShooterLoadTorque = .35;
    // Maximum number of fuel pieces tracked by the desktop simulation.
    inline constexpr int kSimulationCapacity = 54;
    // Number of fuel pieces emitted in one simulated burst.
    inline constexpr int kSimulationBurstSize = 4;
    // Time between simulated burst emissions.
    inline constexpr double kSimulationBurstPeriodSeconds = .20;
    // Nominal fuel feed rate derived from the burst size and burst period.
    inline constexpr double kSimulationFeedBallsPerSecond =
        kSimulationBurstSize / kSimulationBurstPeriodSeconds;
}

namespace season::simulation::motorconfig
{
  // Nominal voltage used to scale motor simulation models.
  inline constexpr double kNominalVoltage = 12.;
  // NEO motor specifications at nominal voltage.
  inline constexpr double kNeoStallCurrentAmps = 105, kNeoStallTorqueNm = 2.6,
                          kNeoFreeSpeedRpm = 5676;
  // Kraken motor specifications at nominal voltage.
  inline constexpr double kKrakenStallCurrentAmps = 366, kKrakenStallTorqueNm = 7.09,
                          kKrakenFreeSpeedRpm = 6000;
  // Largest elapsed interval accepted by the motor simulation update.
  inline constexpr double kMaxUpdateSeconds = 1;
  // Maximum internal physics integration step for simulated motors.
  inline constexpr double kMaxPhysicsStepSeconds = .0002;
  // Speed below which the motor model treats load direction as indeterminate.
  inline constexpr double kLoadDirectionSpeedThreshold = .01;
  // Viscous drag coefficient applied to simulated motor velocity.
  inline constexpr double kViscousDrag = .0001;
}

namespace season::simulation::runtimeconfig
{
  // Number of control-loop iterations to wait after a simulation reset before feedback resumes.
  inline constexpr int kResetFeedbackLoops = 5;
  // Minimum visible AprilTags required for simulated QuestNav alignment.
  inline constexpr int kQuestAlignmentMinTags = 2;
  // Seed for repeatable randomized fuel simulation behavior.
  inline constexpr unsigned kFuelRandomSeed = 3737;
  // Period used by the desktop robot simulation scheduler.
  inline constexpr double kSimulationPeriodSeconds = .02;
  // Horizontal and vertical field of view modeled for the simulated Limelight.
  inline constexpr double kLimelightHorizontalFovDegrees = 62.5, kLimelightVerticalFovDegrees = 48.9;
  // Horizontal and vertical field of view modeled for simulated QuestNav tag detection.
  inline constexpr double kQuestHorizontalFovDegrees = 90, kQuestVerticalFovDegrees = 90;
  // Maximum range at which simulated vision reports targets.
  inline constexpr double kVisionRangeMetres = 6;
  // Position noise scale applied to simulated Limelight tag observations.
  inline constexpr double kVisionTagDistanceNoiseMetres = .75;
  // Standard deviations for simulated Limelight pose measurements (metres for X/Y, radians for heading).
  inline constexpr double kLimelightStdX = .08, kLimelightStdY = .08, kLimelightStdHeading = .12;
  // Standard deviations for simulated QuestNav pose measurements (metres for X/Y, radians for heading).
  inline constexpr double kQuestStdX = .1, kQuestStdY = .1, kQuestStdHeading = .05;
  // Extra clearance used when checking robot spawn positions against the ramp.
  inline constexpr double kRampClearancePaddingMetres = .005;
  // Width of the practice-field spawn lane in inches.
  inline constexpr double kSpawnLaneWidthInches = 50.34;
  // Field X coordinate used to place the simulated robot at spawn.
  inline constexpr double kSpawnXMetres = 3.65;
  // Robot footprint dimensions used for spawn collision checks.
  inline constexpr double kFootprintHalfLengthMetres = geometry::kLengthMetres / 2,
                          kFootprintHalfWidthMetres = geometry::kWidthMetres / 2,
                          kFootprintHeightMetres = .45;
  // Minimum gap required between spawned robot and obstacles.
  inline constexpr double kSpawnClearanceMetres = .02;
  // Duration of the practice autonomous period.
  inline constexpr int kPracticeAutonomousSeconds = 20;
  // Duration of the practice assessment period.
  inline constexpr int kPracticeAssessmentSeconds = 3;
  // Duration of the practice teleoperated period.
  inline constexpr int kPracticeTeleopSeconds = 140;
  // Battery model capacity and voltage parameters.
  inline constexpr double kBatteryCapacityAh = 18.0, kBatteryEmptyVoltage = 10.5,
                          kBatteryVoltageRange = 2.1, kBatteryInternalResistanceOhms = .020;
}

#ifndef __FRC_ROBORIO__
namespace season::simulation
{
  inline base::sim::RobotContactConfig RobotContacts()
  {
    // Conversion factor used to express the robot-contact estimates in metres.
    constexpr double inchesToMetres = .0254;
    base::sim::RobotContactConfig c;
    // Unmeasured construction/material estimates: replace with loaded measurements.
    c.chassisClearance = 2.0 * inchesToMetres;
    c.panThickness = .125 * inchesToMetres;
    c.panHalfLength = geometry::kLengthMetres / 2 - 3.5 * inchesToMetres;
    c.panHalfWidth = geometry::kWidthMetres / 2 - 3.5 * inchesToMetres;
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
