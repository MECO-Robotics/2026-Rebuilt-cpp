#include "season/simulation/DriveSimulationConfig.h"

#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include "base/sim/SimulationTuning.h"
#include <array>
#include <frc/system/plant/DCMotor.h>

namespace season::simulation
{
  base::DriveConfig ConfigureDrive(base::DriveConfig c)
  {
    // Ready-to-drive robot, bumpers, and battery. The real robot keeps its own values.
    c.massKg = 67.6;
    c.cof = 2.1;
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
    std::array<base::sim::ModuleLocation, 4> locations{};
    for (std::size_t i = 0; i < locations.size(); ++i)
      locations[i] = {c.modules[i].xMeters, c.modules[i].yMeters};
    const auto tuned = base::sim::DeriveSimulationTuning(
        {c.massKg, c.moi, c.cof, c.wheelRadius, c.driveRatio, c.slipCurrent, motor.Kt.value(), locations, .08, .50});
    c.autoTranslationKP = tuned.autoTranslationKP;
    c.autoRotationKP = tuned.autoRotationKP;
    return c;
  }
}
#endif
