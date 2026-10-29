#include "season/AutoDrive.h"
#include "season/PerceptionGeometry.h"
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include <base/FootprintAutoDrive.h>
#endif

namespace season {
#ifndef __FRC_ROBORIO__
namespace {
const char* FootprintOutcomeName(base::FootprintAutoDriveOutcome outcome) {
  using O=base::FootprintAutoDriveOutcome;
  switch (outcome) {
    case O::running:return "running";
    case O::arrived:return "arrived";
    case O::stalled:return "stalled";
    case O::blocked:return "blocked";
    case O::no_route:return "no-route";
    case O::planner_busy:return "planner-busy";
    case O::target_unavailable:return "target-unavailable";
    case O::invalid_feedback:return "invalid-feedback";
    case O::source_error:return "source-error";
    case O::settle_failed:return "settle-failed";
    case O::interrupted:return "interrupted";
  }
  return "unknown";
}
}
#endif
void AutoDrive::Status(const char* value) {
  nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Status").SetString(value);
}
frc::Transform3d AutoDrive::Mount(size_t camera) const {
  const std::string prefix=camera==0 ? "Perception/Front/" : "Perception/Rear/";
  // AutoDrive's nominal fore/aft chassis-edge locations; height is an estimate.
  const frc::Transform3d local{
      units::meter_t{frc::Preferences::GetDouble(prefix+"X",camera==0 ? .43 : -.43)},
      units::meter_t{frc::Preferences::GetDouble(prefix+"Y",0)},
      units::meter_t{frc::Preferences::GetDouble(prefix+"Z",.54)},
      frc::Rotation3d{units::degree_t{frc::Preferences::GetDouble(prefix+"RollDeg",0)},
          units::degree_t{frc::Preferences::GetDouble(prefix+"PitchDeg",15)},
          units::degree_t{frc::Preferences::GetDouble(prefix+"YawDeg",camera==0 ? 0 : 180)}}};
  return camera==0 ? HopperCameraMount(extension_(),local) : local;
}
AutoDrive::AutoDrive(base::Drive& drive, std::function<double()> extension,
    std::function<const MatchContext&()> context)
    : matchContext_(std::move(context)),drive_(drive),extension_(std::move(extension)) {
  std::vector<base::ObjectCamera> cameras;
  for(size_t i=0;i<2;++i) {
    const std::string name=i==0 ? "front" : "rear";
    const std::string prefix=i==0 ? "Perception/Front/" : "Perception/Rear/";
    for(const auto& [key,value] : std::vector<std::pair<std::string,double>>{
        {"X",i==0 ? .43 : -.43},{"Y",0},{"Z",.54},{"RollDeg",0},{"PitchDeg",15},{"YawDeg",i==0 ? 0 : 180}})
      frc::Preferences::InitDouble(prefix+key,value);
    std::unique_ptr<base::ObjectDetectionIO> io;
#ifndef __FRC_ROBORIO__
    if(frc::RobotBase::IsSimulation()) {
      meco::sim::PieceSensorConfig config;config.maxRange=6;config.seed=3737+i;
      config.robotToCamera=Mount(i);
      sensors_[i]=std::make_unique<meco::sim::SimulatedPieceSensor>(config);
      io=std::make_unique<base::InjectedObjectDetectionIO>([this,i] {return std::exchange(frames_[i],std::nullopt);});
    } else
#endif
      io=base::MakeNtObjectDetectionIO("/Perception/"+name+"/detections");
    cameras.push_back({name,std::move(io),[this,i]()->std::optional<frc::Transform3d> {return Mount(i);}});
  }
  base::ObjectPerceptionConfig perceptionConfig;
  // Fuse duplicate camera observations without chaining adjacent FUEL into
  // one field-wide centroid (the simulation places centres 0.25 m apart).
  perceptionConfig.clusterRadius=units::meter_t{.15};
  perception_=std::make_unique<base::ObjectPerception>(std::move(cameras),[this] {return frc::Pose3d{drive_.Pose()};},perceptionConfig);
  frc::SmartDashboard::PutNumber("AutoDrive/TargetX",3.0);
  frc::SmartDashboard::PutNumber("AutoDrive/TargetY",.65);
  frc::SmartDashboard::PutNumber("AutoDrive/TargetHeading",0.0);
  frc::SmartDashboard::PutBoolean("AutoDrive/FollowFuel",false);
#ifndef __FRC_ROBORIO__
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintSim",false);
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintProfileSim",false);
  frc::SmartDashboard::PutBoolean("AutoDrive/AvoidBumpsSim",false);
#endif
  nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(false);
  Status("idle-native");
}
void AutoDrive::PublishMatchContext() const {
  const auto& match=MatchState();
  frc::SmartDashboard::PutString("AutoDrive/MatchPhase",match.phase);
  frc::SmartDashboard::PutNumber("AutoDrive/MatchTime",match.periodTime.value_or(-1));
  frc::SmartDashboard::PutBoolean("AutoDrive/HubKnown",match.ownActive.has_value());
  frc::SmartDashboard::PutBoolean("AutoDrive/HubActive",match.ownActive.value_or(false));
  frc::SmartDashboard::PutBoolean("AutoDrive/HubOrderKnown",match.gameDataValid);
  frc::SmartDashboard::PutString("AutoDrive/InactiveFirstAlliance",AllianceLabel(match.inactiveFirstAlliance));
  frc::SmartDashboard::PutString("AutoDrive/FirstActiveAlliance",AllianceLabel(match.firstActiveAlliance));
  frc::SmartDashboard::PutBoolean("AutoDrive/HubNextChangeKnown",match.ownNextChange.has_value());
  frc::SmartDashboard::PutNumber("AutoDrive/HubNextChangeSeconds",match.ownNextChange.value_or(-1));
}
std::optional<frc::Pose2d> AutoDrive::Target() {
  if(frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel",false)) {
    const auto& clusters=perception_->Clusters();
    frc::SmartDashboard::PutBoolean("AutoDrive/DetectionFresh",!clusters.empty());
    if(clusters.empty()) {Status("no-visible-fuel");return std::nullopt;}
    const auto nearest=std::min_element(clusters.begin(),clusters.end(),[this](const auto& a,const auto& b) {
      return a.position.Distance(drive_.Pose().Translation())<b.position.Distance(drive_.Pose().Translation());
    });
    frc::SmartDashboard::PutNumber("AutoDrive/SelectedFuelX",nearest->position.X().value());
    frc::SmartDashboard::PutNumber("AutoDrive/SelectedFuelY",nearest->position.Y().value());
    return frc::Pose2d{nearest->position,(nearest->position-drive_.Pose().Translation()).Angle()};
  }
  return frc::Pose2d{units::meter_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetX",3)},
      units::meter_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetY",.65)},
      frc::Rotation2d{units::radian_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetHeading",0)}}};
}
frc2::CommandPtr AutoDrive::Command() {
  auto sampled=std::make_shared<std::optional<frc::Pose2d>>();
  auto target=[this,sampled] {
        auto goal=Target();
        if(!*sampled && goal) *sampled=goal;
        return goal;
      };
  return frc2::cmd::Defer([this,target]() -> frc2::CommandPtr {
#ifndef __FRC_ROBORIO__
        if(frc::RobotBase::IsSimulation() && frc::SmartDashboard::GetBoolean("AutoDrive/UseFootprintSim",false)) {
          base::FootprintAutoDriveConfig config;
          const bool avoidBumps=frc::SmartDashboard::GetBoolean("AutoDrive/AvoidBumpsSim",false);
          config.footprint={.86,.86};
          config.field={0,0,simulation::kFieldLength,simulation::kFieldWidth};
          // The modeled swerve can drift after a rotation if it releases the
          // drivetrain while residual angular speed is still appreciable.
          config.follow.headingTolerance=.02;
          config.follow.angularSettleSpeed=.02;
          config.follow.settleSamples=10;
          config.follow.finalSettleSamples=50;
          config.follow.poseAngularDamping=.5;
          config.follow.usePoseRateForSettling=true;
          if (frc::SmartDashboard::GetBoolean("AutoDrive/UseFootprintProfileSim",false)) {
            config.follow.terrain=simulation::RampTerrain;
            config.follow.terrainBoundaries=simulation::RampBoundaries;
            config.follow.angularStopTolerance=.01;
            config.follow.translationStallTimeout=3;
            config.follow.translationStallRequest=.01;
            // Illustrative simulation-only capability, not a measured robot limit.
            config.follow.maxAcceleration=4;
          }
          auto obstacles=[avoidBumps] {
            std::vector<base::FieldRectangle> boxes;
            if (frc::SmartDashboard::GetBoolean("FieldSimulation/CollidersEnabled",true)) for (const auto& body:simulation::FieldColliders()) {
              // Skip traversable inclined ramps and objects entirely above or
              // below the modeled chassis contact volume. Remaining field
              // boxes are axis aligned in this desktop model.
              if (std::abs(body.pose.rotation.x)>1e-8 || std::abs(body.pose.rotation.y)>1e-8 ||
                  std::abs(body.pose.rotation.z)>1e-8 ||
                  body.pose.position.z+body.halfExtents.z<.05 ||
                  body.pose.position.z-body.halfExtents.z>.45) continue;
              boxes.push_back({body.pose.position.x-body.halfExtents.x,body.pose.position.y-body.halfExtents.y,
                               body.pose.position.x+body.halfExtents.x,body.pose.position.y+body.halfExtents.y});
            }
            if (frc::SmartDashboard::GetBoolean("FieldSimulation/CollidersEnabled",true) && avoidBumps) {
              const auto ramps=simulation::RampObstacles();
              boxes.insert(boxes.end(),ramps.begin(),ramps.end());
            }
            // Desktop HAL/NT tests can inject one perceived obstacle. It is
            // not a simulated rigid body or a hardware detection source.
            const auto observed=nt::NetworkTableInstance::GetDefault().GetEntry("/Simulation/FootprintObstacle").GetDoubleArray({});
            if (!observed.empty()) {
              if (observed.size()!=4) throw std::runtime_error("invalid simulated footprint obstacle");
              boxes.push_back({observed[0],observed[1],observed[2],observed[3]});
            }
            return boxes;
          };
          auto outcome=std::make_shared<base::FootprintAutoDriveOutcome>();
          auto entry=nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/FootprintOutcome");
          entry.SetString("running");
          return base::AutoDriveFootprintToPose(drive_,target,std::move(obstacles),config,outcome)
              .FinallyDo([entry,outcome](bool) mutable {entry.SetString(FootprintOutcomeName(*outcome));});
        }
#endif
        return base::AutoDriveToPose(drive_,target);
      },{&drive_})
      .BeforeStarting([this,sampled] {
        sampled->reset();
        nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(true);Status("planning-native");
      })
      // Retain the drivetrain until the stop has had time to settle; reporting
      // arrival immediately after the last commanded output can observe coasting.
      .AndThen(frc2::cmd::Wait(units::second_t{.3}))
      .FinallyDo([this,sampled](bool interrupted) {
        nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(false);
        const auto& goal=*sampled;
        const bool reached=goal && AutoDriveArrived(drive_.Pose(),drive_.Speeds(),*goal);
        Status(interrupted ? "interrupted" : reached ? "arrived" : goal ? "stopped-before-goal" : "stopped-no-target");
      });
}
#ifndef __FRC_ROBORIO__
void AutoDrive::UpdateSimulation(const frc::Pose3d& truth, const FuelModel& fuel, bool fieldColliders) {
  std::vector<frc::Translation3d> points;
  points.reserve(fuel.FloorFuel().size());
  for(const auto& p:fuel.FloorFuel()) points.emplace_back(units::meter_t{p.position.x},units::meter_t{p.position.y},units::meter_t{p.position.z});
  const auto obstacles=fieldColliders ? simulation::FieldColliders() : std::vector<base::sim::BodySpec>{};
  ++sequence_;
  for(size_t i=0;i<2;++i) {
    sensors_[i]->SetMount(Mount(i));
    frames_[i]=sensors_[i]->ObserveFrame(truth,points,obstacles,frc::Timer::GetFPGATimestamp(),sequence_);
    frc::SmartDashboard::PutNumber(i==0 ? "AutoDrive/SimFrontDetections" : "AutoDrive/SimRearDetections",frames_[i]->detections.size());
    frc::SmartDashboard::PutNumber(i==0 ? "AutoDrive/SimFrontFrameAge" : "AutoDrive/SimRearFrameAge",(frc::Timer::GetFPGATimestamp()-frames_[i]->captureTime).value());
  }
  const auto& clusters=perception_->Clusters();
  frc::SmartDashboard::PutBoolean("AutoDrive/DetectionFresh",!clusters.empty());
  frc::SmartDashboard::PutNumber("AutoDrive/ClusterCount",clusters.size());
  double nearest=-1;
  for(const auto& cluster:clusters) {
    const double distance=cluster.position.Distance(drive_.Pose().Translation()).value();
    if(nearest<0 || distance<nearest) nearest=distance;
  }
  frc::SmartDashboard::PutNumber("AutoDrive/SimNearestFuelDistance",nearest);
}
void AutoDrive::ResetSimulation() {
  for(auto& frame:frames_) frame.reset();
  perception_->Clear();
}
#endif
}
