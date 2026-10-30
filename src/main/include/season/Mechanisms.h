#pragma once
#include "base/Mechanism.h"
#include "season/Shooting.h"
#include "base/Drive.h"
#include <functional>
#include "base/SystemCheck.h"
#include <utility>

namespace season::hardware
{
  // Java 2026-Rebuilt hardware values; CAN bus strings are significant.
  struct FlywheelConfig
  {
    base::MotorConfig motor;
    double inertiaKgMetersSquared;
  };
  inline const FlywheelConfig kTopIndexer{{{32}, {false}, 1, 30, ""}, .025};
  inline const FlywheelConfig kBottomIndexer{{{31}, {false}, 1, 30, "MECO 2"}, .025};
  inline const FlywheelConfig kConveyor{{{23}, {true}, 1, 30, ""}, .025};
  inline const FlywheelConfig kShooter{{{34, 35}, {false, true}, 22.0 / 14, 40, "MECO 2"}, 24 * .45359237 * .0254 * .0254};
  inline const FlywheelConfig kIntake{{{22, 53}, {true, true}, 1.5, 40, ""}, .025};
  inline const base::Gains kIndexerGains{.maxVelocity = 1000, .maxAcceleration = 1000};
  inline const base::Gains kConveyorGains{.v = .065, .maxVelocity = 1000, .maxAcceleration = 1000};
  inline const base::Gains kShooterGains{.p = .8, .d = .01, .s = .33, .v = .19, .a = .15, .maxVelocity = 1000, .maxAcceleration = 100, .tolerance = .5};
  inline const base::Gains kIntakeGains{.maxVelocity = 1000, .maxAcceleration = 1, .tolerance = .1};
  inline const base::Gains kHoodGains{.p = 20, .s = .5, .gravity = .1, .maxVelocity = 4, .maxAcceleration = 100, .tolerance = .002, .minimum = kHoodMin, .maximum = kHoodMax};
  inline const base::Gains kRackGains{.p = 175, .s = .4, .gravity = .03, .maxVelocity = .5, .maxAcceleration = 100, .tolerance = .02, .minimum = 0, .maximum = .35};
  // These joints use inertia-only simulation plants without gravitational load.
  inline base::Gains SimulationGainsFor(base::Gains gains)
  {
    gains.gravity = 0;
    return gains;
  }
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  inline constexpr double kHoodGearRatio = 105;
  inline constexpr double kRackGearRatio = 9 / (kPi * .0254);
  inline constexpr double kRackDrumRadius = .5 * .0254;
  inline constexpr double kRackInputInertia = 15 * .45359237 * kRackDrumRadius * kRackDrumRadius / (kRackGearRatio * kRackGearRatio);
  inline constexpr double kRackStow = 0, kRackDeploy = .30, kRackSafe = .12;
  inline constexpr double kIntakeVolts = -10, kIntakeSlowVolts = 7, kEjectVolts = 10;
  inline constexpr double kConveyorFeedVolts = -11, kBottomFeedVolts = -10, kTopFeedVolts = 10;
  inline constexpr double kHubHood = 0, kFerryHood = .049, kTrenchHood = .049;
  inline constexpr double kHubRps = 30, kFerryRps = 55, kTrenchRps = 35;
  inline constexpr double kAutoReadyTimeoutSeconds = 2, kAutoFeedSeconds = 1, kAutoTotalTimeoutSeconds = 3;
}

namespace season
{
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
  frc2::CommandPtr Acquire(Mechanisms m);
  frc2::CommandPtr Eject(Mechanisms m);
  frc2::CommandPtr Deploy(Mechanisms m);
  frc2::CommandPtr Stow(Mechanisms m);
  // One-shot presets intentionally retain mechanism control after command ends.
  frc2::CommandPtr ShooterPreset(Mechanisms m, double hoodRotations, double flywheelRps);
  frc2::CommandPtr ShooterIdle(Mechanisms m);
  frc2::CommandPtr DriverHubPreset(Mechanisms m);
  frc2::CommandPtr AgitateAndFeed(Mechanisms m, std::function<void(double)> rumble = [](double) {});
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
    { return 30.; };
    std::function<void(bool)> readyFeedback = [](bool) {};
    bool autonomous = false, simulation = false;
    // Markers share the path drivetrain and must not claim or command it.
    bool ownDrive = true;
    double readyTimeoutSeconds = 2, feedSeconds = 1, totalTimeoutSeconds = 3;
  };
  frc2::CommandPtr CoordinatedShot(Mechanisms m, base::Drive &drive, FieldGeometry field,
                                   std::function<ShotTarget()> target, CoordinatedShotOptions options = {});
}


namespace season
{
    // Register unloaded pit checks only. Caller supplies ALL motors to neutralize in manager callback.
    // Shooter intentionally runs at 10 rps; no practice-shot/feed sequence is registered.
    void ConfigureSystemChecks(base::SystemCheck &check, base::Drive &drive,
                               base::PositionJoint &rack, base::PositionJoint &hood,
                               base::Flywheel &shooter,
                               std::vector<std::pair<std::string, base::Flywheel *>> rollers,
                               std::function<bool()> visionConnected);
}
