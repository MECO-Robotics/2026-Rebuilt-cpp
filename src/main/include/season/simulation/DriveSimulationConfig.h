#pragma once

#include "base/RuntimeTuning.h"

#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include <meco/sim/DriveTuning.h>
#include <utility>
#include <frc/system/plant/DCMotor.h>

namespace season::simulation
{
  namespace driveconfig
  {
    // Desktop plant estimates and assumed center-of-mass height.
    inline constexpr double kRobotMassKg = 67.6;
    inline constexpr double kWheelCoefficientOfFriction = 2.1;
    inline constexpr double kWheelInertiaKgMetresSquared = .01;
    inline constexpr double kSteerInertiaKgMetresSquared = .01;
    inline constexpr double kCenterOfMassHeightMetres = .30;
    // Error scales used by automatic drive controller tuning.
    inline constexpr double kTranslationErrorScaleMetres = .08;
    inline constexpr double kHeadingErrorScaleRadians = .50;
  }

  // Apply the desktop plant and controller assumptions before constructing Drive.
  inline base::DriveConfig ConfigureDrive(base::DriveConfig c, base::SimulationTuning tuning)
  {
    // Ready-to-drive robot, bumpers, and battery. The real robot keeps its own values.
    c.massKg = driveconfig::kRobotMassKg;
    c.cof = driveconfig::kWheelCoefficientOfFriction;
    c.simulation.bumperHalfLength = geometry::kLengthMetres / 2;
    c.simulation.bumperHalfWidth = geometry::kWidthMetres / 2;
    c.simulation.bumperWallThickness = fuelconfig::kBumperDepthMetres;
    c.simulation.bumperCornerRadius = simulationconfig::kBumperCornerRadiusMetres;
    c.simulation.bumperFriction = simulationconfig::kBumperSurfaceFriction;
    c.simulation.rollInertia = simulationconfig::kRollInertiaKgMetresSquared;
    c.simulation.pitchInertia = simulationconfig::kPitchInertiaKgMetresSquared;
    c.simulation.centerOfMassX = 0;
    c.simulation.centerOfMassY = 0;
    c.simulation.wheelInertia = driveconfig::kWheelInertiaKgMetresSquared;
    c.simulation.steerInertia = driveconfig::kSteerInertiaKgMetresSquared;
    c.simulation.longitudinalStiffness = simulationconfig::kTireLongitudinalStiffness;
    c.simulation.lateralStiffness = simulationconfig::kTireLateralStiffness;
    c.simulation.wheelWidth = simulationconfig::kWheelWidthMetres;
    c.simulation.contactPatchLength = simulationconfig::kTireContactPatchLengthMetres;
    c.simulation.contactShearDeflection = simulationconfig::kTireShearDeflectionMetres;

    // Bumper envelope uses the season contact geometry; preserve the assumed
    // world center of mass at 0.30 m above the floor.
    const auto contact = RobotContacts();
    c.simulation.bumperHalfHeight = contact.noodleDiameter;
    c.simulation.rideHeight = contact.bumperFloorGap + c.simulation.bumperHalfHeight;
    c.simulation.centerOfMassZ = driveconfig::kCenterOfMassHeightMetres - c.simulation.rideHeight;

    // Rotor and encoder mounting axes are independent of electrical inversion.
    using Orientation = base::SimulationDeviceOrientation;
    c.simulation.modules = {
        {{Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive}}};

    // Use the Phoenix FOC motor model and stator torque limit of the simulator.
    const auto motor = frc::DCMotor::KrakenX60FOC(1);
    const auto motorProfile = [&](double currentLimit)
    {
      return base::DriveMotorSimulationConfig{
          .resistanceOhms = motor.R.value(),
          .torqueConstantNmPerAmp = motor.Kt.value(),
          .velocityConstantRadPerSecPerVolt = motor.Kv.value(),
          .currentLimitAmps = currentLimit,
          .frictionVoltage = simulationconfig::kMotorFrictionVoltage};
    };
    c.simulation.driveMotor = motorProfile(c.slipCurrent);
    c.simulation.steerMotor = motorProfile(c.steerCurrent);
    return meco::sim::ConfigureDriveTuning(std::move(c), tuning, motor.Kt.value(),
                                            driveconfig::kTranslationErrorScaleMetres,
                                            driveconfig::kHeadingErrorScaleRadians);
  }
}
#endif
