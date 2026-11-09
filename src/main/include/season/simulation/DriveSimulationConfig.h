#pragma once

#include "base/RuntimeTuning.h"

#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include <meco/sim/DriveTuning.h>
#include <utility>
#include <frc/system/plant/DCMotor.h>

namespace season::simulation
{
  // Apply the desktop plant and controller assumptions before constructing Drive.
  inline base::DriveConfig ConfigureDrive(base::DriveConfig c, base::SimulationTuning tuning)
  {
    // Ready-to-drive robot, bumpers, and battery. The real robot keeps its own values.
    c.massKg = 67.6;
    c.cof = 2.1;
    c.simulation.bumperHalfLength = geometry::kLengthMetres / 2;
    c.simulation.bumperHalfWidth = geometry::kWidthMetres / 2;
    c.simulation.bumperWallThickness = fuelconfig::kBumperDepthMetres;
    c.simulation.wheelInertia = .01;
    c.simulation.steerInertia = .01;

    // Bumper envelope uses the season contact geometry; preserve the assumed
    // world center of mass at 0.30 m above the floor.
    const auto contact = RobotContacts();
    c.simulation.bumperHalfHeight = contact.noodleDiameter;
    c.simulation.rideHeight = contact.bumperFloorGap + c.simulation.bumperHalfHeight;
    c.simulation.centerOfMassZ = .30 - c.simulation.rideHeight;

    // Rotor and encoder mounting axes are independent of electrical inversion.
    using Orientation = base::SimulationDeviceOrientation;
    c.simulation.modules = {
        {{Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::CounterClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive},
         {Orientation::ClockwisePositive, Orientation::ClockwisePositive, Orientation::CounterClockwisePositive}}};

    // Use the Phoenix FOC motor model and stator torque limit of the simulator.
    const auto motor = frc::DCMotor::KrakenX60FOC(1);
    return meco::sim::ConfigureDriveTuning(std::move(c), tuning, motor.Kt.value(), .08, .50);
  }
}
#endif
