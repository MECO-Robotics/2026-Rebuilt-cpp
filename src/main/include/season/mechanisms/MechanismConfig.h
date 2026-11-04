#pragma once

#include "base/Mechanism.h"
#include "season/shooting/Shooting.h"
#include <units/voltage.h>

// Motor wiring, mechanism controller tuning, and physical limits.
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
  // Assumed rotational inertia in kg·m² for lightweight transport rollers in simulation.
  inline constexpr double kTransportRollerInertia = .0002;
  // The first reversal flag sets the primary direction; later flags oppose the primary.
  // An empty CAN bus name selects the default bus.
  // CAN ID, inversion, gearing, current limit, and simulated inertia for the top indexer.
  inline const FlywheelConfig kTopIndexer{
      .motor = {.ids = {32},
                .reversed = {false},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = ""},
      .inertiaKgMetersSquared = kTransportRollerInertia};
  // CAN ID, inversion, gearing, current limit, bus, and inertia for the bottom indexer.
  inline const FlywheelConfig kBottomIndexer{
      .motor = {.ids = {31},
                .reversed = {false},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = "MECO 2"},
      .inertiaKgMetersSquared = kTransportRollerInertia};
  // CAN ID, inversion, gearing, current limit, bus, and inertia for the conveyor.
  inline const FlywheelConfig kConveyor{
      .motor = {.ids = {23},
                .reversed = {true},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = ""},
      .inertiaKgMetersSquared = kTransportRollerInertia};
  // Shooter motor IDs, inversions, gearing, current limit, bus, and modeled wheel inertia.
  inline const FlywheelConfig kShooter{
      .motor = {.ids = {34, 35},
                .reversed = {false, true},
                .gearRatio = 22.0 / 14,
                .currentLimit = 40,
                .canBus = "MECO 2"},
      .inertiaKgMetersSquared = 24 * .45359237 * .0254 * .0254};
  // Intake motor IDs, inversions, gearing, current limit, bus, and modeled roller inertia.
  inline const FlywheelConfig kIntake{
      .motor = {.ids = {22, 53},
                .reversed = {true, true},
                .gearRatio = 1.5,
                .currentLimit = 40,
                .canBus = ""},
      .inertiaKgMetersSquared = kTransportRollerInertia};
  // Motion limits shared by the indexer velocity controllers.
  inline constexpr base::Gains kIndexerGains{
      .maxVelocity = 1000,
      .maxAcceleration = 1000};
  // Conveyor feedforward and motion limits for its velocity controller.
  inline constexpr base::Gains kConveyorGains{
      .v = .065,
      .maxVelocity = 1000,
      .maxAcceleration = 1000};
  // Shooter velocity controller gains, feedforward, motion limits, and ready tolerance.
  inline constexpr base::Gains kShooterGains{
      .p = .8,
      .d = .01,
      .s = .33,
      .v = .19,
      .a = .15,
      .maxVelocity = 1000,
      .maxAcceleration = 100,
      .tolerance = .5};
  // Intake velocity limits and tolerance used by its controller.
  inline constexpr base::Gains kIntakeGains{
      .maxVelocity = 1000,
      .maxAcceleration = 1,
      .tolerance = .1};
  // Hood position-controller tuning, motion limits, tolerance, and allowed travel range.
  inline constexpr base::Gains kHoodGains{
      .p = 20,
      .s = .5,
      .gravity = .1,
      .maxVelocity = 4,
      .maxAcceleration = 100,
      .tolerance = .002,
      .minimum = season::kHoodMin,
      .maximum = season::kHoodMax};
  // Rack position-controller tuning, motion limits, tolerance, and extension bounds.
  inline constexpr base::Gains kRackGains{
      .p = 175,
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
  // CAN device IDs for the hood and rack motor controllers, respectively.
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  // Hood motor rotations per hood output rotation.
  inline constexpr double kHoodGearRatio = 105;
  // Rack motor rotations per metre of linear extension.
  inline constexpr double kRackGearRatio = 9 / (season::kPi * .0254);
  // Radius in metres of the rack drum that converts rotation to extension.
  inline constexpr double kRackDrumRadius = .5 * .0254;
}
