#include "season/detail/RuntimeAdapters.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "season/mechanisms/Mechanisms.h"
#include <base/RuntimeMode.h>
#ifndef __FRC_ROBORIO__
#include "season/simulation/SimulationConfig.h"
#endif

namespace season::runtime
{
  namespace
  {
    PolicyProfile ConfiguredPolicy()
    {
      PolicyProfile profile;
      profile.behavior.emptyQuietSeconds = offenseconfig::kEmptyQuietSeconds;
      profile.behavior.shotTimeoutSeconds = shotconfig::kShotTimeoutSeconds;
      return profile;
    }
  }

  const PolicyProfile &Policy()
  {
    return base::SelectRuntimeIO<const PolicyProfile &>(
        []() -> const PolicyProfile & { static const auto profile = ConfiguredPolicy(); return profile; },
        []() -> const PolicyProfile & {
          static const auto profile = [] {
            auto profile = ConfiguredPolicy();
#ifndef __FRC_ROBORIO__
            profile.autonomous = {true, true};
            profile.shot = {HubSpeedModel::BallisticOpening, 0.};
            profile.behavior.defaultCapacity = fuelconfig::kSimulationCapacity;
            profile.behavior.defaultFeedRate = fuelconfig::kSimulationFeedBallsPerSecond;
            profile.behavior.emptyQuietSeconds = fuelconfig::kSimulationBurstPeriodSeconds;
            profile.behavior.emptyShooterCurrentAmps = simulationconfig::kEmptyShooterCurrentAmps;
            profile.behavior.emptyRollerNoiseAmps = simulationconfig::kEmptyRollerNoiseAmps;
            profile.behavior.shotTimeoutSeconds = simulationconfig::kShotTimeoutSeconds;
            profile.behavior.transportActive = simulation::TransportFeeding;
#endif
            return profile;
          }();
          return profile;
        },
        []() -> const PolicyProfile & { static const auto profile = ConfiguredPolicy(); return profile; });
  }

  Behavior RuntimeBehavior() { return Policy().behavior; }
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
