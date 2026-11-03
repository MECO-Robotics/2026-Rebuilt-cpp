#pragma once
#include "base/Drive.h"
#include "season/RobotControlConfig.h"
#include "season/simulation/SimulationConfig.h"
#include <array>
#include <frc/RobotBase.h>
#include <frc/system/plant/DCMotor.h>
#include <ctre/phoenix6/CANBus.hpp>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationTuning.h"
#endif

namespace season::hardware
{
  // Requested odometry update rates by CAN network type.
  inline constexpr double kSwerveOdometryFdHz = 500, kSwerveOdometryClassicHz = 100;
  using namespace units::literals;
  inline base::DriveConfig DriveConfig()
  {
    base::DriveConfig c;
    c.bus = "MECO CANIvore";
    c.pigeon = 13;
    c.odometryHz = frc::RobotBase::IsSimulation() || ctre::phoenix6::CANBus{c.bus}.IsNetworkFD()
                       ? kSwerveOdometryFdHz
                       : kSwerveOdometryClassicHz;
    c.controlPeriodSeconds = kControlPeriod.value();
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
}
