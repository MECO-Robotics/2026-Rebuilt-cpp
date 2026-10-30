#include "season/OffensiveAutoDrive.h"
#include <base/AutoDrive.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <algorithm>
#include <cmath>
#include <numbers>

using namespace units::literals;
namespace season {
namespace {
void Phase(const char* phase) { frc::SmartDashboard::PutString("AutoDrive/OffensePhase", phase); }
}
frc2::CommandPtr OffensiveAutoDrive(AutoDrive& perception, base::Drive& drive,
                                    Mechanisms m, FieldGeometry field) {
  auto collect = [&perception, &drive, m, field]() {
    auto goal = [&perception, &drive, field]() -> std::optional<frc::Pose2d> {
      const auto alliance = frc::DriverStation::GetAlliance();
      if (!alliance) return std::nullopt;
      auto points = perception.VisibleFuel();
      const auto& match = perception.MatchState();
      // Upstream avoids collecting in its own zone while its hub is inactive.
      if (!match.ownActive.value_or(false))
        std::erase_if(points, [alliance,field](const auto& p) { return InAllianceZone(*alliance,p,field); });
      if (points.empty()) return std::nullopt;
      const auto pose=drive.Pose();
      const auto nearest=std::min_element(points.begin(),points.end(),[&](const auto& a,const auto& b) {
        return a.Distance(pose.Translation()) < b.Distance(pose.Translation());
      });
      const auto direction=(*nearest-pose.Translation()).Angle();
      // Drive the intake through the observation, not just the chassis centre
      // to it. The planner remains responsible for reaching the resulting pose.
      return frc::Pose2d{*nearest + frc::Translation2d{.55_m,direction},direction};
    };
    return frc2::cmd::Deadline(frc2::cmd::Wait(7_s),
        base::AutoDriveTrackTarget(drive,goal), Acquire(m))
        .BeforeStarting([] { Phase("collect"); });
  };
  auto home = [&drive,field]() -> std::optional<frc::Pose2d> {
    auto alliance=frc::DriverStation::GetAlliance();
    if (!alliance) return std::nullopt;
    bool red=*alliance==frc::DriverStation::Alliance::kRed;
    double x=red ? field.length-3.65 : 3.65;
    double y=red ? field.width-.75 : .75;
    auto target=frc::Translation2d{units::meter_t{x},units::meter_t{y}};
    auto bearing=(HubPosition(*alliance)-target).Angle();
    return frc::Pose2d{target,bearing};
  };
  auto returnHome = [&drive,home]() {
    return base::AutoDriveToPose(drive,home).WithTimeout(12_s)
        .BeforeStarting([] {Phase("return");});
  };
  auto shot = [&perception,&drive,m,field]() {
    CoordinatedShotOptions options;
    options.ownDrive=true;
    options.autonomous=true;
    options.simulation=frc::RobotBase::IsSimulation();
    options.readyTimeoutSeconds=8;
    options.feedSeconds=8;
    options.totalTimeoutSeconds=15;
    auto target=[] {
      auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
      return ShotTarget{ShotMode::Hub,HubPosition(alliance)};
    };
    return CoordinatedShot(m,drive,field,target,options)
        .Until([&perception] {return !perception.MatchState().ownActive.value_or(false);})
        .BeforeStarting([] {Phase("score");});
  };
  auto waitForHub = [&perception]() {
    return frc2::cmd::WaitUntil([&perception] {
      return frc::DriverStation::GetAlliance().has_value() && perception.MatchState().ownActive.value_or(false);
    }).BeforeStarting([] {Phase("wait-for-hub");});
  };
  auto cycle=frc2::cmd::Sequence(collect(),returnHome(),waitForHub(),shot()).Repeatedly();
  return frc2::cmd::Sequence(waitForHub(),shot(),std::move(cycle))
      .FinallyDo([&drive,m](bool) {
        drive.Stop();m.rack.Stop();m.intake.Stop();m.conveyor.Stop();m.bottomIndexer.Stop();m.topIndexer.Stop();m.hood.Stop();m.shooter.Stop();Phase("idle");
      });
}
}
