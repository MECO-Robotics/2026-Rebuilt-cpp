#include "season/OffensiveAutoDrive.h"
#include <base/AutoDrive.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
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
  // Anonymous detections are only a rough pickup estimate: an observation
  // disappearing in the intake mouth may also be occluded. Never treat this
  // estimate as a measured hopper sensor or as a capacity guarantee.
  auto estimate=std::make_shared<OffensiveFuelEstimate>();
  frc::Preferences::InitDouble("AutoDrive/EstimatedCapacity",0);

  auto collect = [&perception, &drive, m, field, estimate]() {
    struct TargetLock {std::optional<frc::Translation2d> position; double seen=0;};
    auto lock=std::make_shared<TargetLock>();
    auto goal = [&perception, &drive, field, lock, estimate]() -> std::optional<frc::Pose2d> {
      const auto alliance = frc::DriverStation::GetAlliance();
      if (!alliance || !perception.MatchState().ownActive.has_value()) return std::nullopt;
      auto points = perception.VisibleFuel();
      const auto pose=drive.Pose();
      estimate->Observe(points,pose);
      frc::SmartDashboard::PutNumber("AutoDrive/EstimatedFuel",estimate->Count());
      const auto& match = perception.MatchState();
      // Upstream avoids collecting in its own zone while its hub is inactive.
      if (!match.ownActive.value_or(false))
        std::erase_if(points, [alliance,field](const auto& p) { return InAllianceZone(*alliance,p.position,field); });
      const double now=frc::Timer::GetFPGATimestamp().value();
      if (lock->position && !*match.ownActive && InAllianceZone(*alliance,*lock->position,field)) lock->position.reset();
      if (lock->position) {
        const auto found=std::find_if(points.begin(),points.end(),[&](const auto& p) { return p.position.Distance(*lock->position).value()<.3; });
        if (found!=points.end()) {lock->position=found->position;lock->seen=now;}
        else if (now-lock->seen>.4) lock->position.reset();
      }
      if (points.empty() && !lock->position) {
        // No observations: seek the neutral zone so the cameras can reacquire.
        const frc::Translation2d search{units::meter_t{field.length/2},
            units::meter_t{field.width/2}};
        return frc::Pose2d{search,(search-pose.Translation()).Angle()};
      }
      // Upstream ranks four nearby candidates by estimated pickup-and-return
      // time per nearby piece when active, and by distance while staging.
      // Candidate age and confidence come from the perception snapshot.
      const auto intake=pose.Translation()+frc::Translation2d{.58_m,pose.Rotation()};
      std::sort(points.begin(),points.end(),[&](const auto& a,const auto& b) {
        return a.position.Distance(intake)<b.position.Distance(intake);
      });
      const auto count=std::min<std::size_t>(4,points.size());
      // Upstream's acceleration-limited pickup + return cycle time divided
      // by confidence-weighted nearby yield (no hidden simulator state).
      const auto speeds=frc::ChassisSpeeds::FromRobotRelativeSpeeds(drive.Speeds(),pose.Rotation());
      const double maxSpeed=std::min(2.,drive.Config().maxSpeed), acceleration=3., maxOmega=3.;
      auto travelTime=[&](double distance,double initial) {
        initial=std::clamp(initial,0.,maxSpeed);
        double toMax=(maxSpeed*maxSpeed-initial*initial)/(2*acceleration);
        return distance<=toMax ? (std::sqrt(initial*initial+2*acceleration*distance)-initial)/acceleration :
          (maxSpeed-initial)/acceleration+(distance-toMax)/maxSpeed;
      };
      auto cost=[&](const base::ObjectCluster& candidate) {
        const auto delta=candidate.position-intake;
        const double outbound=delta.Norm().value();
        const double age=std::clamp(now-candidate.captureTime.value(),0.,2.);
        if (!*match.ownActive) return outbound+age*.4;
        double yield=0;
        for (const auto& other:points) if (candidate.position.Distance(other.position).value()<=1.22)
          yield+=std::clamp(other.confidence,0.,1.)*std::exp(-std::max(0.,now-other.captureTime.value()));
        const double projected=outbound>1e-6 ? (speeds.vx.value()*delta.X().value()+speeds.vy.value()*delta.Y().value())/outbound : 0.;
        const double turn=std::abs(std::remainder((delta.Angle()-pose.Rotation()).Radians().value(),2*std::numbers::pi))/maxOmega;
        const double boundary=AllianceBoundary(*alliance,field);
        const double back=*alliance==frc::DriverStation::Alliance::kBlue ?
            std::max(0.,candidate.position.X().value()-boundary) : std::max(0.,boundary-candidate.position.X().value());
        return (std::max(travelTime(outbound,projected),turn)+travelTime(back,0)+.2+age*.4/maxSpeed)/std::max(.1,yield);
      };
      if (!lock->position) {
        auto chosen=std::min_element(points.begin(),points.begin()+count,[&](const auto& a,const auto& b) {return cost(a)<cost(b);});
        lock->position=chosen->position;lock->seen=now;
      }
      const auto direction=(*lock->position-pose.Translation()).Angle();
      const auto through=*lock->position + frc::Translation2d{.55_m,direction};
      return frc::Pose2d{frc::Translation2d{units::meter_t{std::clamp(through.X().value(),.5,field.length-.5)},units::meter_t{std::clamp(through.Y().value(),.5,field.width-.5)}},direction};
    };
    return frc2::cmd::Deadline(frc2::cmd::Wait(20_s),
        base::AutoDriveTrackTarget(drive,goal).AndThen(frc2::cmd::Wait(.3_s)).Repeatedly(), Acquire(m))
        .Until([&perception,estimate] {const double capacity=frc::Preferences::GetDouble("AutoDrive/EstimatedCapacity",0);return (std::isfinite(capacity) && capacity>0 && estimate->Count()>=capacity) || !frc::DriverStation::GetAlliance().has_value() || !perception.MatchState().ownActive.has_value();})
        .BeforeStarting([lock,estimate] {lock->position.reset();estimate->StartCollection();Phase("collect"); });
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
  auto returnHome = [&perception,&drive,home]() {
    return base::AutoDriveToPose(drive,home).WithTimeout(12_s)
        .Until([&perception] {return !perception.MatchState().ownActive.value_or(false) || !frc::DriverStation::GetAlliance().has_value();})
        .BeforeStarting([] {Phase("return");});
  };
  auto shot = [&perception,&drive,m,field,estimate]() {
    CoordinatedShotOptions options;
    options.ownDrive=true;
    options.autonomous=true;
    options.simulation=frc::RobotBase::IsSimulation();
    options.readyTimeoutSeconds=8;
    options.feedSeconds=8;
    options.totalTimeoutSeconds=15;
    auto fed=std::make_shared<bool>(false);
    options.readyFeedback=[fed](bool ready) {if (ready) *fed=true;};
    auto target=[] {
      auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
      return ShotTarget{ShotMode::Hub,HubPosition(alliance)};
    };
    return CoordinatedShot(m,drive,field,target,options)
        .Until([&perception] {return !perception.MatchState().ownActive.value_or(false);})
        .BeforeStarting([fed] {*fed=false;Phase("score");})
        .FinallyDo([estimate,fed](bool interrupted) {estimate->CompleteFeed(!interrupted && *fed);});
  };
  auto ferry = [&perception,&drive,m,field,estimate]() {
    CoordinatedShotOptions options;
    options.ownDrive=true; options.autonomous=true;
    options.simulation=frc::RobotBase::IsSimulation();
    options.readyTimeoutSeconds=8; options.feedSeconds=8; options.totalTimeoutSeconds=15;
    auto fed=std::make_shared<bool>(false);
    options.readyFeedback=[fed](bool ready) {if (ready) *fed=true;};
    auto target=[&drive,field] {
      auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
      return FerryTarget(alliance,drive.Pose(),field);
    };
    return CoordinatedShot(m,drive,field,target,options)
      .Until([&perception] {return !perception.MatchState().ownActive.has_value() || *perception.MatchState().ownActive;})
      .BeforeStarting([fed] {*fed=false;Phase("ferry");})
      .FinallyDo([estimate,fed](bool interrupted) {estimate->CompleteFeed(!interrupted && *fed);});
  };
  auto waitForKnown = [&perception]() {
    return frc2::cmd::WaitUntil([&perception] {
      return frc::DriverStation::GetAlliance().has_value() && perception.MatchState().ownActive.has_value();
    }).BeforeStarting([] {Phase("wait-for-match");});
  };
  auto active = [&perception] {return perception.MatchState().ownActive.value_or(false);};
  auto mayShoot = [&perception,&drive,home] {
    const auto target=home();
    return OffensiveHubShotAllowed(perception.MatchState().ownActive,target,drive.Pose(),drive.Speeds());
  };
  auto cycle=frc2::cmd::Sequence(waitForKnown(),collect(),waitForKnown(),
      frc2::cmd::Either(frc2::cmd::Sequence(returnHome(),
          frc2::cmd::Either(shot(),frc2::cmd::None(),mayShoot)),ferry(),active)).Repeatedly();
  return std::move(cycle).FinallyDo([&drive,m](bool) {
        drive.Stop();m.rack.Stop();m.intake.Stop();m.conveyor.Stop();m.bottomIndexer.Stop();m.topIndexer.Stop();m.hood.Stop();m.shooter.Stop();Phase("idle");
      });
}
}
