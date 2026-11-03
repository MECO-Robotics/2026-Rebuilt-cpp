#pragma once
#include "base/Drive.h"
#include "base/QuestNav.h"
#include "base/Vision.h"
#include "season/drive/FastControlProfile.h"
#include "season/simulation/RobotContactConfig.h"
#include "base/Mechanism.h"
#include "season/shooting/Shooting.h"
#include <units/voltage.h>
#include <cmath>
#include <array>
#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/system/plant/DCMotor.h>
#include <ctre/phoenix6/CANBus.hpp>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationTuning.h"
#endif

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
}

namespace season::hardware
{
  using namespace units::literals;
  // Configure the swerve hardware, geometry, and simulation model.
  inline base::DriveConfig DriveConfig(bool fastControlEnabled)
  {
    base::DriveConfig c;
    c.bus = "MECO CANIvore";
    c.pigeon = 13;
    const auto &profile = season::drive::fastcontrol::Config();
    c.odometryHz = frc::RobotBase::IsSimulation() || ctre::phoenix6::CANBus{c.bus}.IsNetworkFD()
                       ? profile.odometryFdHz
                       : profile.odometryClassicHz;
    if (fastControlEnabled)
    {
      c.controlPeriodSeconds = profile.periodMs / 1000.0;
      c.externalSimulationScheduling = true;
    }
    c.driveRatio = 5.8909090909090915;
    c.steerRatio = 12.1;
    c.couplingRatio = 4.909090909090909;
    c.maxSpeed = 5.24;
    c.maxAngularRate = 7;
    c.driveKP = .15;
    c.driveKS = .1;
    c.driveKV = .124;
    const bool simulation = frc::RobotBase::IsSimulation();
    c.steerKP = 100;
    c.steerKD = .5;
    c.steerKS = .2;
    c.steerKV = 1.5;
    c.driveCurrent = 55;
    c.slipCurrent = 50;
    c.steerCurrent = 30;
    // Simulation ready-to-drive maximum: robot, bumpers and battery (67.6 kg).
    // The real configuration remains unchanged. The yaw MOI is the existing
    // PathPlanner estimate; replace it when a measured or CAD value is available.
    c.massKg = simulation ? 67.6 : 53;
    c.moi = 6.883;
    c.cof = simulation ? 2.1 : 2.9;
    c.simulation.wheelInertia = .01;
    c.simulation.steerInertia = .01;
    if (simulation)
    {
      // Bumper envelope: user-confirmed 0.5 inch floor gap, estimated 5.5 inch height.
      // Move the body origin to bumper centre; preserve the assumed world CoM at .30 m.
#ifndef __FRC_ROBORIO__
      const auto contact = season::simulation::RobotContacts();
      c.simulation.bumperHalfHeight = contact.noodleDiameter;
      c.simulation.rideHeight = contact.bumperFloorGap + c.simulation.bumperHalfHeight;
#endif
      c.simulation.centerOfMassZ = .30 - c.simulation.rideHeight;
    }
    c.autoConfigFromGUI = true;
    c.autoTranslationKP = 15;
    c.autoRotationKP = 10;
    // Module order is front-left, front-right, back-left, back-right.
    c.modules = {{{2, 1, 9, .3076171875, .2794, .2794, false, true, false},
                  {8, 3, 10, .304931640625, .2794, -.2794, true, true, false},
                  {6, 5, 11, -.083740234375, -.2794, .2794, false, true, false},
                  {4, 7, 12, -.346435546875, -.2794, -.2794, true, true, false}}};
    // Physical mounting directions measured from the current module geometry.
    // These describe the simulated rotor/encoder axes, independently of the
    // Phoenix electrical inversion settings above.
    using Orientation = base::SimulationDeviceOrientation;
    c.simulation.modules = {
        {{Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive}}};
#ifndef __FRC_ROBORIO__
    if (simulation)
    {
      // Native Phoenix swerve requests enable FOC. Use the same electrical
      // plant and stator torque limit as the simulation adapter, not supply amps.
      const auto motor = frc::DCMotor::KrakenX60FOC(1);
      const double motorTorqueConstant = motor.Kt.value();
      std::array<base::sim::ModuleLocation, 4> locations{};
      for (std::size_t i = 0; i < locations.size(); ++i)
        locations[i] = {c.modules[i].xMeters, c.modules[i].yMeters};
      const auto tuned = base::sim::DeriveSimulationTuning(
          {c.massKg, c.moi, c.cof, c.wheelRadius, c.driveRatio, c.slipCurrent, motorTorqueConstant, locations, .08, .50});
      // The contact-loaded plant uses the configured robot steering gains.
      // The old simulation-only pole estimate assumed an unloaded azimuth motor.
      c.autoTranslationKP = tuned.autoTranslationKP;
      c.autoRotationKP = tuned.autoRotationKP;
    }
#endif
    return c;
  }
  // Express the headset mount relative to the robot frame.
  inline base::QuestNavConfig QuestConfig(const frc::AprilTagFieldLayout &layout)
  {
    base::QuestNavConfig c;
    c.subscriptionPeriodSeconds = season::drive::fastcontrol::Config().questPeriodMs / 1000.0;
    c.fieldLength = layout.GetFieldLength().value();
    c.fieldWidth = layout.GetFieldWidth().value();
    c.robotToQuest = frc::Transform3d{units::meter_t{-13.5 * .0254}, 0_m, units::meter_t{8.5 * .0254},
                                      frc::Rotation3d{180_deg, 0_deg, 180_deg}};
    return c;
  }
  // Express the Limelight mount relative to the robot frame.
  inline base::VisionConfig VisionConfig(const frc::AprilTagFieldLayout &layout)
  {
    base::VisionConfig c;
    c.name = "limelight";
    c.fieldLength = layout.GetFieldLength().value();
    c.fieldWidth = layout.GetFieldWidth().value();
    c.robotToCamera = frc::Transform3d{0_m, 0_m, units::meter_t{21.25 * .0254}, frc::Rotation3d{0_deg, -30_deg, 0_deg}};
    return c;
  }
}
