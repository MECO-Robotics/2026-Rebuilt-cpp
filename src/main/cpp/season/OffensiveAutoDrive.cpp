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
      if (!alliance || !perception.MatchState().ownActive.has_value()) return std::nullopt;
      auto points = perception.VisibleFuel();
      const auto& match = perception.MatchState();
      // Upstream avoids collecting in its own zone while its hub is inactive.
      if (!match.ownActive.value_or(false))
        std::erase_if(points, [alliance,field](const auto& p) { return InAllianceZone(*alliance,p,field); });
      if (points.empty()) return std::nullopt;
      const auto pose=drive.Pose();
      // Upstream ranks four nearby candidates by estimated pickup-and-return
      // time per nearby piece when active, and by distance while staging.
      // Our perception exposes positions only, not track age or confidence.
      const auto intake=pose.Translation()+frc::Translation2d{.58_m,pose.Rotation()};
      std::sort(points.begin(),points.end(),[&](const auto& a,const auto& b) {
        return a.Distance(intake)<b.Distance(intake);
      });
      const auto count=std::min<std::size_t>(4,points.size());
      auto cost=[&](const frc::Translation2d& p) {
        const double outbound=p.Distance(intake).value();
        if (!*match.ownActive) return outbound;
        double yield=0;
        for (const auto& other:points) if (p.Distance(other).value()<=1.22) ++yield;
        const double boundary=AllianceBoundary(*alliance,field);
        const double back=*alliance==frc::DriverStation::Alliance::kBlue ?
            std::max(0.,p.X().value()-boundary) : std::max(0.,boundary-p.X().value());
        return (outbound+back+.2)/std::max(1.,yield);
      };
      auto chosen=std::min_element(points.begin(),points.begin()+count,[&](const auto& a,const auto& b) {return cost(a)<cost(b);});
      const auto direction=(*chosen-pose.Translation()).Angle();
      const auto through=*chosen + frc::Translation2d{.55_m,direction};
      return frc::Pose2d{frc::Translation2d{units::meter_t{std::clamp(through.X().value(),.5,field.length-.5)},units::meter_t{std::clamp(through.Y().value(),.5,field.width-.5)}},direction};
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
  auto ferry = [&perception,&drive,m,field]() {
    CoordinatedShotOptions options;
    options.ownDrive=true; options.autonomous=true;
    options.simulation=frc::RobotBase::IsSimulation();
    options.readyTimeoutSeconds=8; options.feedSeconds=8; options.totalTimeoutSeconds=15;
    auto target=[&drive,field] {
      auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
      return FerryTarget(alliance,drive.Pose(),field);
    };
    return CoordinatedShot(m,drive,field,target,options)
      .Until([&perception] {return !perception.MatchState().ownActive.has_value() || *perception.MatchState().ownActive;})
      .BeforeStarting([] {Phase("ferry");});
  };
  auto waitForKnown = [&perception]() {
    return frc2::cmd::WaitUntil([&perception] {
      return frc::DriverStation::GetAlliance().has_value() && perception.MatchState().ownActive.has_value();
    }).BeforeStarting([] {Phase("wait-for-match");});
  };
  auto active = [&perception] {return perception.MatchState().ownActive.value_or(false);};
  auto cycle=frc2::cmd::Sequence(waitForKnown(),collect(),waitForKnown(),
      frc2::cmd::Either(frc2::cmd::Sequence(returnHome(),shot()),ferry(),active)).Repeatedly();
  return std::move(cycle).FinallyDo([&drive,m](bool) {
        drive.Stop();m.rack.Stop();m.intake.Stop();m.conveyor.Stop();m.bottomIndexer.Stop();m.topIndexer.Stop();m.hood.Stop();m.shooter.Stop();Phase("idle");
      });
}
}
