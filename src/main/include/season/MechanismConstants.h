#pragma once
#include "base/Mechanism.h"
#include "season/Shooting.h"
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
  inline const base::Gains kHoodGains{.p = 20, .s = .5, .gravity = .1, .maxVelocity = 4, .maxAcceleration = 100, .tolerance = .002, .minimum = 0, .maximum = .049};
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
