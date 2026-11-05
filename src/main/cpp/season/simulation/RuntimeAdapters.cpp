#include "season/detail/RuntimeAdapters.h"
#include "season/detail/RuntimePolicy.h"
#include "base/SimulationLatency.h"
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#ifndef __FRC_ROBORIO__
#include <meco/sim/PracticeMatchController.h>
#endif

namespace season::runtime
{
  double ObserveForwardInput(double value) { return base::simulation::ObserveJoystick(value); }

  AutonomousDefaults AutoDefaults() { return Policy().autonomous; }

  ShotCalibration ShootingCalibration() { return Policy().shot; }

  std::optional<frc2::CommandPtr> PracticeControl()
  {
#ifndef __FRC_ROBORIO__
    return meco::sim::TryStartPracticeMatch();
#else
    return std::nullopt;
#endif
  }
}
