#include "season/detail/RuntimeBehavior.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "season/mechanisms/Mechanisms.h"
#include <base/RuntimeMode.h>
#include <frc/RobotBase.h>
#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#include "season/simulation/TransportFeedback.h"
#endif

namespace season::runtime
{
  Behavior RuntimeBehavior()
  {
    Behavior behavior;
    behavior.emptyQuietSeconds = offenseconfig::kEmptyQuietSeconds;
    behavior.shotTimeoutSeconds = shotconfig::kShotTimeoutSeconds;
#ifndef __FRC_ROBORIO__
    if (base::CurrentRuntimeMode() == base::Mode::Sim)
    {
      behavior.defaultCapacity = fuelconfig::kSimulationCapacity;
      behavior.defaultFeedRate = fuelconfig::kSimulationFeedBallsPerSecond;
      behavior.emptyQuietSeconds = fuelconfig::kSimulationBurstPeriodSeconds;
      behavior.emptyShooterCurrentAmps = simulationconfig::kEmptyShooterCurrentAmps;
      behavior.emptyRollerNoiseAmps = simulationconfig::kEmptyRollerNoiseAmps;
      behavior.shotTimeoutSeconds = simulationconfig::kShotTimeoutSeconds;
      behavior.transportActive = simulation::TransportFeeding;
    }
#endif
    return behavior;
  }
}
#ifndef __FRC_ROBORIO__
namespace season::simulation
{
  bool TransportFeeding(const Mechanisms &m)
  {
    return m.bottomIndexer.Inputs().voltage < -simulationconfig::kSimTransportVoltageThreshold &&
           m.topIndexer.Inputs().voltage > simulationconfig::kSimTransportVoltageThreshold &&
           m.conveyor.Inputs().voltage < -simulationconfig::kSimTransportVoltageThreshold &&
           m.bottomIndexer.Velocity() < 0 && m.topIndexer.Velocity() > 0 && m.conveyor.Velocity() < 0;
  }
}
#endif
