#pragma once
#include "base/Mechanism.h"
#include "season/shooting/ShootingConfig.h"
#include <units/voltage.h>
#include <cmath>

namespace season
{
  inline constexpr double kRackAngleRadians = 7.5 * kPi / 180;
  inline const double kRackAngleCos = std::cos(kRackAngleRadians);
  inline const double kRackAngleSin = std::sin(kRackAngleRadians);
}
namespace season::hardware
{
  inline constexpr double kShooterPivotXMetres = -0.103310, kShooterPivotZMetres = 0.423;
  inline constexpr double kClimberPivotXMetres = -0.290, kClimberPivotZMetres = 0.512;
  inline constexpr int kPilotControllerPort = 0, kCopilotControllerPort = 1;
  // Java 2026-Rebuilt hardware values; CAN bus strings are significant.
  // Season motor configuration and modeled rotational inertia.
  struct FlywheelConfig
  {
    base::MotorConfig motor;
    double inertiaKgMetersSquared;
  };
  // Estimated output inertia for lightweight transport rollers. This is a
  // simulation assumption, not a measured CAD value; it gives rapid reversals.
  inline constexpr double kSimTransportVoltageThreshold = 1;
  inline constexpr double kTransportRollerInertia = .0002;
  inline const FlywheelConfig kTopIndexer{{{32}, {false}, 1, 30, ""}, kTransportRollerInertia};
  inline const FlywheelConfig kBottomIndexer{{{31}, {false}, 1, 30, "MECO 2"}, kTransportRollerInertia};
  inline const FlywheelConfig kConveyor{{{23}, {true}, 1, 30, ""}, kTransportRollerInertia};
  inline const FlywheelConfig kShooter{{{34, 35}, {false, true}, 22.0 / 14, 40, "MECO 2"},
                                       24 * .45359237 * .0254 * .0254};
  inline const FlywheelConfig kIntake{{{22, 53}, {true, true}, 1.5, 40, ""}, kTransportRollerInertia};
  inline constexpr base::Gains kIndexerGains{.maxVelocity = 1000, .maxAcceleration = 1000};
  inline constexpr base::Gains kConveyorGains{.v = .065, .maxVelocity = 1000, .maxAcceleration = 1000};
  inline constexpr base::Gains kShooterGains{
      .p = .8, .d = .01, .s = .33, .v = .19, .a = .15, .maxVelocity = 1000, .maxAcceleration = 100, .tolerance = .5};
  inline constexpr base::Gains kIntakeGains{.maxVelocity = 1000, .maxAcceleration = 1, .tolerance = .1};
  inline constexpr base::Gains kHoodGains{.p = 20,
                                      .s = .5,
                                      .gravity = .1,
                                      .maxVelocity = 4,
                                      .maxAcceleration = 100,
                                      .tolerance = .002,
                                      .minimum = kHoodMin,
                                      .maximum = kHoodMax};
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
  inline constexpr units::second_t kControlPeriod{.02};
  inline constexpr units::volt_t kBrownoutVoltage{6.3};
  inline constexpr int kRackMotorCurrent = 40, kHoodMotorCurrent = 40;
  inline constexpr double kRackMotorInertia = 15 * .45359237 * .0127 * .0127;
  inline constexpr double kHoodMotorInertia = .01;
  inline constexpr double kAutoIntakeConveyorVolts = 5;
  inline constexpr double kAcquireConveyorVolts = 5, kAcquireBottomVolts = 5, kAcquireTopVolts = -5;
  inline constexpr double kEjectConveyorVolts = -5, kEjectBottomVolts = 10, kEjectTopVolts = -10;
  inline constexpr double kStowConveyorVolts = 10;
  inline constexpr double kShotRetractSpeedMps = .10, kShotHoldPositionMetres = .13;
  inline constexpr double kShotIntakeAcceleration = 100, kHubReadyTimeoutSeconds = 5;
  inline constexpr double kMovingFerryMaxSpeed = 1., kStationaryShotMaxSpeed = .25;
  inline constexpr double kMovingFerryMaxAngularSpeed = .3, kStationaryShotMaxAngleRadians = 10 * kPi / 180;
  inline constexpr double kShotVelocityToleranceRps = 3, kRumbleFeedIntensity = .35;
  inline constexpr double kRumblePulseIntensity = .75, kRumblePeriodSeconds = 1.;
  inline constexpr double kRumbleFirstPulseEndSeconds = .15, kRumbleSecondPulseStartSeconds = .30;
  inline constexpr double kRumbleSecondPulseEndSeconds = .45;
  inline constexpr double kDriverDeadband = .1, kAimHeadingGain = 5;
  inline constexpr double kAutoAimHeadingGain = 7, kAutoAimMaxAngularRate = 7;
  inline constexpr double kShotReadyTimeoutSeconds = 8, kShotSimTimeoutSeconds = 25, kShotRealTimeoutSeconds = 15;
  inline constexpr double kVisionFreshnessSeconds = .5;
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  inline constexpr double kHoodGearRatio = 105;
  inline constexpr double kRackGearRatio = 9 / (kPi * .0254);
  inline constexpr double kRackDrumRadius = .5 * .0254;
  inline constexpr double kRackInputInertia =
      15 * .45359237 * kRackDrumRadius * kRackDrumRadius / (kRackGearRatio * kRackGearRatio);
  inline constexpr double kRackStow = 0, kRackDeploy = .30, kRackSafe = .12;
  inline constexpr double kIntakeVolts = -10, kIntakeSlowVolts = 7, kEjectVolts = 10;
  inline constexpr double kConveyorFeedVolts = -11, kBottomFeedVolts = -10, kTopFeedVolts = 10;
  inline constexpr double kHubHood = 0, kFerryHood = .049, kTrenchHood = .049;
  inline constexpr double kHubRps = 30, kFerryRps = 55, kTrenchRps = 35;
  inline constexpr double kAutoReadyTimeoutSeconds = 2, kAutoFeedSeconds = 1, kAutoTotalTimeoutSeconds = 3;
} // namespace season::hardware
