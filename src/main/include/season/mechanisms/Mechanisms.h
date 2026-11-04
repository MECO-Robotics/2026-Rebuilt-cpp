#pragma once
#include "base/Mechanism.h"
#include "season/shooting/Shooting.h"
#include "base/Drive.h"
#include <functional>
#include "base/SystemCheck.h"
#include <utility>
#include <units/voltage.h>


namespace season::hardware
{
  // Requested control resend rates in Hz for Spark, Talon FX FD, and classic Talon controllers, respectively.
  inline constexpr double kSparkControlHz = 100, kTalonFdControlHz = 250, kTalonClassicControlHz = 100;
  // Requested status refresh rates in Hz for Spark, Talon FX FD, and classic Talon controllers, respectively.
  inline constexpr double kSparkStatusHz = 100, kTalonFdStatusHz = 250, kTalonClassicStatusHz = 100;
  // Java 2026-Rebuilt hardware values; CAN bus strings are significant.
  // Season motor configuration and modeled rotational inertia.
  struct FlywheelConfig
  {
    base::MotorConfig motor;
    double inertiaKgMetersSquared;
  };
  // Minimum voltage magnitude used by simulation to consider a transport motor commanded.
  inline constexpr double kSimTransportVoltageThreshold = 1;
  // Assumed rotational inertia in kg·m² for lightweight transport rollers in simulation.
  inline constexpr double kTransportRollerInertia = .0002;
  // CAN ID, inversion, gearing, current limit, and simulated inertia for the top indexer.
  inline const FlywheelConfig kTopIndexer{{{32}, {false}, 1, 30, ""}, kTransportRollerInertia};
  // CAN ID, inversion, gearing, current limit, bus, and inertia for the bottom indexer.
  inline const FlywheelConfig kBottomIndexer{{{31}, {false}, 1, 30, "MECO 2"}, kTransportRollerInertia};
  // CAN ID, inversion, gearing, current limit, bus, and inertia for the conveyor.
  inline const FlywheelConfig kConveyor{{{23}, {true}, 1, 30, ""}, kTransportRollerInertia};
  // Shooter motor IDs, inversions, gearing, current limit, bus, and modeled wheel inertia.
  inline const FlywheelConfig kShooter{{{34, 35}, {false, true}, 22.0 / 14, 40, "MECO 2"},
                                       24 * .45359237 * .0254 * .0254};
  // Intake motor IDs, inversions, gearing, current limit, bus, and modeled roller inertia.
  inline const FlywheelConfig kIntake{{{22, 53}, {true, true}, 1.5, 40, ""}, kTransportRollerInertia};
  // Motion limits shared by the indexer velocity controllers.
  inline constexpr base::Gains kIndexerGains{.maxVelocity = 1000, .maxAcceleration = 1000};
  // Conveyor feedforward and motion limits for its velocity controller.
  inline constexpr base::Gains kConveyorGains{.v = .065, .maxVelocity = 1000, .maxAcceleration = 1000};
  // Shooter velocity controller gains, feedforward, motion limits, and ready tolerance.
  inline constexpr base::Gains kShooterGains{
      .p = .8, .d = .01, .s = .33, .v = .19, .a = .15, .maxVelocity = 1000, .maxAcceleration = 100, .tolerance = .5};
  // Intake velocity limits and tolerance used by its controller.
  inline constexpr base::Gains kIntakeGains{.maxVelocity = 1000, .maxAcceleration = 1, .tolerance = .1};
  // Hood position-controller tuning, motion limits, tolerance, and allowed travel range.
  inline constexpr base::Gains kHoodGains{.p = 20,
                                      .s = .5,
                                      .gravity = .1,
                                      .maxVelocity = 4,
                                      .maxAcceleration = 100,
                                      .tolerance = .002,
                                      .minimum = season::kHoodMin,
                                      .maximum = season::kHoodMax};
  // Rack position-controller tuning, motion limits, tolerance, and extension bounds.
  inline constexpr base::Gains kRackGains{.p = 175,
                                      .s = .4,
                                      .gravity = .03,
                                      .maxVelocity = .5,
                                      .maxAcceleration = 100,
                                      .tolerance = .02,
                                      .minimum = 0,
                                      .maximum = .35};
  // These joints use inertia-only simulation plants without gravitational load.
  inline base::Gains SimulationGainsFor(base::Gains gains)
  {
    gains.gravity = 0;
    return gains;
  }
  // Voltage below which the robot reports brownout state.
  inline constexpr units::volt_t kBrownoutVoltage{6.3};
  // Current limits in amperes for the rack and hood motors, respectively.
  inline constexpr int kRackMotorCurrent = 40, kHoodMotorCurrent = 40;
  // Estimated rack and hood motor inertias in kg·m² used by their simulation plants, respectively.
  inline constexpr double kRackMotorInertia = 15 * .45359237 * .0127 * .0127, kHoodMotorInertia = .01;
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
  // Maximum heading error in radians allowed before a stationary shot proceeds.
  inline constexpr double kStationaryShotMaxAngleRadians = 10 * season::kPi / 180;
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
  // Joystick magnitude below this fraction is treated as zero by driver controls.
  inline constexpr double kDriverDeadband = .1;
  // Heading-error proportional gain used by manual aiming control.
  inline constexpr double kAimHeadingGain = 5;
  // Heading-error proportional gain used by autonomous aiming control.
  inline constexpr double kAutoAimHeadingGain = 7;
  // Maximum angular rate in rad/s commanded by autonomous aiming.
  inline constexpr double kAutoAimMaxAngularRate = 7;
  // Maximum time in seconds for a shot to reach its ready state.
  inline constexpr double kShotReadyTimeoutSeconds = 8;
  // Maximum total shot-command durations in seconds for desktop simulation and the real robot, respectively.
  inline constexpr double kShotSimTimeoutSeconds = 25, kShotRealTimeoutSeconds = 15;
  // Maximum age in seconds for a vision observation to be considered fresh.
  inline constexpr double kVisionFreshnessSeconds = .5;
  // CAN device IDs for the hood and rack motor controllers, respectively.
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  // Hood motor rotations per hood output rotation.
  inline constexpr double kHoodGearRatio = 105;
  // Rack motor rotations per metre of linear extension.
  inline constexpr double kRackGearRatio = 9 / (season::kPi * .0254);
  // Radius in metres of the rack drum that converts rotation to extension.
  inline constexpr double kRackDrumRadius = .5 * .0254;
  // Rack inertia reflected to the motor input through its gearing.
  inline constexpr double kRackInputInertia =
      15 * .45359237 * kRackDrumRadius * kRackDrumRadius / (kRackGearRatio * kRackGearRatio);
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
  // Hood output rotations for the hub, ferry, and trench shooting presets, respectively.
  inline constexpr double kHubHood = 0, kFerryHood = .049, kTrenchHood = .049;
  // Shooter target speeds in RPS for the hub, ferry, and trench presets, respectively.
  inline constexpr double kHubRps = 30, kFerryRps = 55, kTrenchRps = 35;
  // Autonomous shot timing in seconds: readiness wait, fallback feed duration, and total timeout, respectively.
  inline constexpr double kAutoReadyTimeoutSeconds = 2, kAutoFeedSeconds = 1, kAutoTotalTimeoutSeconds = 3;
}

namespace season
{
  // Non-owning references shared by season command factories.
  struct Mechanisms
  {
    base::PositionJoint &rack;
    base::Flywheel &intake;
    base::Flywheel &conveyor;
    base::Flywheel &bottomIndexer;
    base::Flywheel &topIndexer;
    base::PositionJoint &hood;
    base::Flywheel &shooter;
  };
  // Voltage alone does not establish roller direction during a reversal.
  // Used by desktop fuel simulation and its matching feed-time feedback.
  inline bool SimulatedTransportFeeding(Mechanisms m)
  {
    return m.bottomIndexer.Inputs().voltage < -hardware::kSimTransportVoltageThreshold && m.topIndexer.Inputs().voltage > hardware::kSimTransportVoltageThreshold &&
           m.conveyor.Inputs().voltage < -hardware::kSimTransportVoltageThreshold && m.bottomIndexer.Velocity() < 0 &&
           m.topIndexer.Velocity() > 0 && m.conveyor.Velocity() < 0;
  }
  // Full-hopper feed duration from robot preferences, with the desktop model's
  // configured values as simulation defaults. Zero means unconfigured.
  double ConfiguredFeedDurationSeconds();
  // Acquisition commands own and stop their participating mechanisms.
  frc2::CommandPtr Acquire(Mechanisms m);
  frc2::CommandPtr Eject(Mechanisms m);
  frc2::CommandPtr Deploy(Mechanisms m);
  frc2::CommandPtr Stow(Mechanisms m);
  // One-shot presets intentionally retain mechanism control after command ends.
  frc2::CommandPtr ShooterPreset(Mechanisms m, double hoodRotations, double flywheelRps);
  frc2::CommandPtr ShooterIdle(Mechanisms m);
  frc2::CommandPtr DriverHubPreset(Mechanisms m);
  frc2::CommandPtr AgitateAndFeed(Mechanisms m, std::function<void(double)> rumble = [](double) {});
  // Suppliers are sampled by the running shot command.
  struct CoordinatedShotOptions
  {
    std::function<bool()> poseReady = []
    { return true; };
    std::function<bool()> forceFeed = []
    { return false; };
    std::function<bool()> calibration = []
    { return false; };
    std::function<double()> calibrationHood = []
    { return 0.; };
    std::function<double()> calibrationRps = []
    { return hardware::kHubRps; };
    std::function<void(bool)> readyFeedback = [](bool) {};
    // In simulation this reports measured transport activity, not just intent.
    std::function<void(bool)> feedFeedback = [](bool) {};
    bool autonomous = false, simulation = false;
    // Run the intake and retract the rack while a ready volley feeds.
    bool agitate = false;
    // Markers share the path drivetrain and must not claim or command it.
    bool ownDrive = true;
    double readyTimeoutSeconds = hardware::kAutoReadyTimeoutSeconds, feedSeconds = hardware::kAutoFeedSeconds, totalTimeoutSeconds = hardware::kAutoTotalTimeoutSeconds;
    // Optional duration sampled once when the command starts.
    std::function<double()> feedDurationSeconds;
    // Optional feedback-based early completion after feed has started.
    std::function<bool()> feedComplete;
    // Field-relative translation in metres per second while aiming a moving ferry.
    std::function<frc::Translation2d()> ferryTranslation;
  };
  // Autonomous shots end on feed completion or a bounded timeout.
  frc2::CommandPtr CoordinatedShot(Mechanisms m, base::Drive &drive, FieldGeometry field,
                                   std::function<ShotTarget()> target, CoordinatedShotOptions options = {});
} // namespace season

namespace season
{
  // Register unloaded pit checks only. Caller supplies ALL motors to neutralize in manager callback.
  // Shooter intentionally runs at 10 rps; no practice-shot/feed sequence is registered.
  void ConfigureSystemChecks(base::SystemCheck &check, base::Drive &drive, base::PositionJoint &rack,
                             base::PositionJoint &hood, base::Flywheel &shooter,
                             std::vector<std::pair<std::string, base::Flywheel *>> rollers,
                             std::function<bool()> visionConnected);
} // namespace season
