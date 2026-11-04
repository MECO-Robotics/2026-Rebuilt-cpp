#include "season/detail/RuntimeAdapters.h"
#include "base/SimulationLatency.h"
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#ifndef __FRC_ROBORIO__
#include <frc/simulation/DriverStationSim.h>
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
      return frc2::cmd::RunOnce([] {
        const char *station = "Blue1";
        switch (frc::sim::DriverStationSim::GetAllianceStationId())
        {
        case HAL_AllianceStationID_kRed1: station = "Red1"; break;
        case HAL_AllianceStationID_kRed2: station = "Red2"; break;
        case HAL_AllianceStationID_kRed3: station = "Red3"; break;
        case HAL_AllianceStationID_kBlue2: station = "Blue2"; break;
        case HAL_AllianceStationID_kBlue3: station = "Blue3"; break;
        default: break;
        }
        nt::NetworkTableInstance::GetDefault().GetTable("/Sim/Practice")
            ->GetEntry("Config/AllianceStation").SetString(station);
        static auto command = nt::NetworkTableInstance::GetDefault()
                                  .GetStringTopic("/Sim/Practice/Command")
                                  .Publish({.keepDuplicates = true});
        command.Set("start");
      }).IgnoringDisable(true);
#endif
    return std::nullopt;
  }
}
