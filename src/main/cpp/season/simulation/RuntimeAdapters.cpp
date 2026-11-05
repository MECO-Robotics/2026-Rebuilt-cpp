#include "season/detail/RuntimeAdapters.h"
#include "base/SimulationLatency.h"
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#ifndef __FRC_ROBORIO__
#include <meco/sim/PracticeMatchController.h>
#endif

namespace season::runtime
{
  double ObserveForwardInput(double value) { return base::simulation::ObserveJoystick(value); }

  AutonomousDefaults AutoDefaults()
  {
    const bool desktop = frc::RobotBase::IsSimulation();
    return {desktop, desktop};
  }

  ShotCalibration ShootingCalibration()
  {
    return frc::RobotBase::IsSimulation()
               ? ShotCalibration{HubSpeedModel::BallisticOpening, 0.}
               : ShotCalibration{};
  }

  std::optional<frc2::CommandPtr> PracticeControl()
  {
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation())
      return meco::sim::StartPracticeMatch();
#endif
    return std::nullopt;
  }
}
