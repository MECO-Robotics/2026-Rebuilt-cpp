#pragma once

#include "base/Mechanism.h"
#include "season/RobotGeometry.h"
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
  // Role identifies a mechanism when selecting its IO.
  enum class MotorRole { Intake, Conveyor, BottomIndexer, TopIndexer, Shooter };
  enum class JointRole { Rack, Hood };
  // Season motor configuration and role.
  struct FlywheelConfig
  {
    base::MotorConfig motor;
    MotorRole role;
  };
  // The first reversal flag sets the primary direction; later flags oppose the primary.
  // An empty CAN bus name selects the default bus.
  // CAN ID, inversion, gearing, and current limit for the top indexer.
  inline const FlywheelConfig kTopIndexer{
      .motor = {.ids = {32},
                .reversed = {false},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = ""},
      .role = MotorRole::TopIndexer};
  // CAN ID, inversion, gearing, current limit, and bus for the bottom indexer.
  inline const FlywheelConfig kBottomIndexer{
      .motor = {.ids = {31},
                .reversed = {false},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = "MECO 2"},
      .role = MotorRole::BottomIndexer};
  // CAN ID, inversion, gearing, current limit, and bus for the conveyor.
  inline const FlywheelConfig kConveyor{
      .motor = {.ids = {23},
                .reversed = {true},
                .gearRatio = 1,
                .currentLimit = 30,
                .canBus = ""},
      .role = MotorRole::Conveyor};
  // Shooter motor IDs, inversions, gearing, current limit, and bus.
  inline const FlywheelConfig kShooter{
      .motor = {.ids = {34, 35},
                .reversed = {false, true},
                .gearRatio = 22.0 / 14,
                .currentLimit = 40,
                .canBus = "MECO 2"},
      .role = MotorRole::Shooter};
  // Intake motor IDs, inversions, gearing, current limit, and bus.
  inline const FlywheelConfig kIntake{
      .motor = {.ids = {22, 53},
                .reversed = {true, true},
                .gearRatio = 1.5,
                .currentLimit = 40,
                .canBus = ""},
      .role = MotorRole::Intake};
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
  // Voltage below which the robot reports brownout state.
  inline constexpr units::volt_t kBrownoutVoltage{6.3};
  // Current limits in amperes for the rack and hood motors, respectively.
  inline constexpr int kRackMotorCurrent = 40, kHoodMotorCurrent = 40;
  // CAN device IDs for the hood and rack motor controllers, respectively.
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  // Hood motor rotations per hood output rotation.
  inline constexpr double kHoodGearRatio = 105;
  // Rack motor rotations per metre of linear extension.
  inline constexpr double kRackGearRatio = 9 / (season::kPi * season::kMetresPerInch);
  // Radius in metres of the rack drum that converts rotation to extension.
  inline constexpr double kRackDrumRadius = .5 * season::kMetresPerInch;
}
