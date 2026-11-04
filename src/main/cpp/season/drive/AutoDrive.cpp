#include "season/drive/AutoDrive.h"
#include "season/RobotGeometry.h"
#include <frc2/command/Commands.h>
#include <frc/Preferences.h>
#include <frc/Filesystem.h>
#include <base/FootprintAutoDrive.h>
#include <fstream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>


namespace season
{
  std::unique_ptr<base::ObjectDetectionIO> AutoDriveEnvironment::Camera(
      std::size_t, const std::string &name, std::function<frc::Transform3d()>, units::second_t period)
  {
    return base::MakeNtObjectDetectionIO("/Perception/" + name + "/detections", period);
  }

  namespace
  {
    const char *FootprintOutcomeName(base::FootprintAutoDriveOutcome outcome)
    {
      using O = base::FootprintAutoDriveOutcome;
      switch (outcome)
      {
      case O::running:
        return "running";
      case O::arrived:
        return "arrived";
      case O::stalled:
        return "stalled";
      case O::blocked:
        return "blocked";
      case O::no_route:
        return "no-route";
      case O::planner_busy:
        return "planner-busy";
      case O::target_unavailable:
        return "target-unavailable";
      case O::target_unstable:
        return "target-unstable";
      case O::target_changed:
        return "target-changed";
      case O::invalid_feedback:
        return "invalid-feedback";
      case O::source_error:
        return "source-error";
      case O::settle_failed:
        return "settle-failed";
      case O::interrupted:
        return "interrupted";
      }
      return "unknown";
    }
  } // namespace

  // Publish a concise reason for the current navigation state.
  void AutoDrive::Status(const char *value)
  {
    nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Status").SetString(value);
  }
  // Account for the front camera moving with the intake.
  frc::Transform3d AutoDrive::Mount(size_t camera) const
  {
    const std::string prefix = camera == 0 ? "Perception/Front/" : "Perception/Rear/";
    const auto &defaults = camera == 0 ? vision::config::kFrontCamera : vision::config::kRearCamera;
    // AutoDrive's nominal fore/aft chassis-edge locations; height is an estimate.
    const frc::Transform3d local{
        units::meter_t{frc::Preferences::GetDouble(prefix + "X", defaults.xMetres)},
        units::meter_t{frc::Preferences::GetDouble(prefix + "Y", defaults.yMetres)},
        units::meter_t{frc::Preferences::GetDouble(prefix + "Z", defaults.zMetres)},
        frc::Rotation3d{units::degree_t{frc::Preferences::GetDouble(prefix + "RollDeg", defaults.rollDegrees)},
                        units::degree_t{frc::Preferences::GetDouble(prefix + "PitchDeg", defaults.pitchDegrees)},
                        units::degree_t{frc::Preferences::GetDouble(prefix + "YawDeg", defaults.yawDegrees)}}};
    return camera == 0 ? HopperCameraMount(extension_(), local) : local;
  }
  AutoDrive::AutoDrive(base::Drive &drive, std::function<double()> extension,
                       std::function<const base::season2026::MatchContext &()> context,
                       units::second_t detectionUpdatePeriod, std::shared_ptr<AutoDriveEnvironment> environment)
      : matchContext_(std::move(context)), drive_(drive), extension_(std::move(extension)),
        environment_(environment ? std::move(environment) : std::make_shared<AutoDriveEnvironment>())
  {
    std::vector<base::ObjectCamera> cameras;

    for (size_t i = 0; i < 2; ++i)
    {
      const std::string name = i == 0 ? "front" : "rear";
      const std::string prefix = i == 0 ? "Perception/Front/" : "Perception/Rear/";
      const auto &defaults = i == 0 ? vision::config::kFrontCamera : vision::config::kRearCamera;
      for (const auto &[key, value] : std::vector<std::pair<std::string, double>>{
               {"X", defaults.xMetres}, {"Y", defaults.yMetres}, {"Z", defaults.zMetres},
               {"RollDeg", defaults.rollDegrees}, {"PitchDeg", defaults.pitchDegrees},
               {"YawDeg", defaults.yawDegrees}})
        frc::Preferences::InitDouble(prefix + key, value);
      auto io = environment_->Camera(i, name, [this, i] { return Mount(i); }, detectionUpdatePeriod);
      cameras.push_back({name, std::move(io), [this, i]() -> std::optional<frc::Transform3d>
                         { return Mount(i); }});
    }
    base::ObjectPerceptionConfig perceptionConfig;
    // Fuse duplicate camera observations without chaining adjacent FUEL into
    // one field-wide centroid (the simulation places centres 0.25 m apart).
    perceptionConfig.clusterRadius = units::meter_t{vision::config::kClusterRadiusMetres};
    perception_ = std::make_unique<base::ObjectPerception>(
        std::move(cameras), [this]
        { return frc::Pose3d{drive_.Pose()}; }, perceptionConfig);
    frc::SmartDashboard::PutNumber("AutoDrive/TargetX", autodriveconfig::kDefaultTargetXMetres);
    frc::SmartDashboard::PutNumber("AutoDrive/TargetY", autodriveconfig::kDefaultTargetYMetres);
    frc::SmartDashboard::PutNumber("AutoDrive/TargetHeading", 0.0);
    frc::SmartDashboard::PutBoolean("AutoDrive/FollowFuel", false);
    frc::SmartDashboard::PutBoolean("AutoDrive/UseConfiguredFootprint", false);
    // Read once at startup; no file I/O occurs in a scheduler callback.
    const std::string path = frc::filesystem::GetDeployDirectory() + "/autodrive-footprint.json";
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (file)
    {
      const auto size = file.tellg();
      if (size > 0 && size <= autodriveconfig::kMaxGridBytes)
      {
        std::string contents(static_cast<std::size_t>(size), '\0');
        file.seekg(0);
        if (file.read(contents.data(), size))
          configuredFootprint_ = base::ParseFootprintConfiguration(contents);
      }
    }
    frc::SmartDashboard::PutString("AutoDrive/ConfiguredFootprintStatus",
                                   configuredFootprint_ ? "loaded" : "missing-or-invalid");

    nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(false);
    Status("idle-native");
  }
  AutoDrive::~AutoDrive() = default;
  void AutoDrive::PublishMatchContext() const
  {
    const auto &match = MatchState();
    frc::SmartDashboard::PutString("AutoDrive/MatchPhase", match.phase);
    frc::SmartDashboard::PutNumber("AutoDrive/MatchTime", match.periodTime.value_or(-1));
    frc::SmartDashboard::PutBoolean("AutoDrive/HubKnown", match.ownActive.has_value());
    frc::SmartDashboard::PutBoolean("AutoDrive/HubActive", match.ownActive.value_or(false));
    frc::SmartDashboard::PutBoolean("AutoDrive/HubOrderKnown", match.gameDataValid);
    frc::SmartDashboard::PutString("AutoDrive/InactiveFirstAlliance",
                                   base::season2026::AllianceLabel(match.inactiveFirstAlliance));
    frc::SmartDashboard::PutString("AutoDrive/FirstActiveAlliance",
                                   base::season2026::AllianceLabel(match.firstActiveAlliance));
    frc::SmartDashboard::PutBoolean("AutoDrive/HubNextChangeKnown", match.ownNextChange.has_value());
    frc::SmartDashboard::PutNumber("AutoDrive/HubNextChangeSeconds", match.ownNextChange.value_or(-1));
  }
  // Return the perception snapshot, not simulation ground truth.
  const std::vector<base::ObjectCluster> &AutoDrive::VisibleFuel() const { return perception_->Clusters(); }
  // Follow visible fuel or use the fixed blue-origin dashboard target.
  std::optional<frc::Pose2d> AutoDrive::Target()
  {
    if (frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false))
    {
      const auto &clusters = perception_->Clusters();
      frc::SmartDashboard::PutBoolean("AutoDrive/DetectionFresh", !clusters.empty());
      if (clusters.empty())
      {
        Status("no-visible-fuel");
        return std::nullopt;
      }
      const auto nearest = std::min_element(clusters.begin(), clusters.end(),
                                            [this](const auto &a, const auto &b)
                                            {
                                              return a.position.Distance(drive_.Pose().Translation()) <
                                                     b.position.Distance(drive_.Pose().Translation());
                                            });
      frc::SmartDashboard::PutNumber("AutoDrive/SelectedFuelX", nearest->position.X().value());
      frc::SmartDashboard::PutNumber("AutoDrive/SelectedFuelY", nearest->position.Y().value());
      return frc::Pose2d{nearest->position, (nearest->position - drive_.Pose().Translation()).Angle()};
    }
    return frc::Pose2d{units::meter_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetX", autodriveconfig::kDefaultTargetXMetres)},
                       units::meter_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetY", autodriveconfig::kDefaultTargetYMetres)},
                       frc::Rotation2d{units::radian_t{frc::SmartDashboard::GetNumber("AutoDrive/TargetHeading", 0)}}};
  }
  // Select the configured navigation mode when the command starts.
  frc2::CommandPtr AutoDrive::Command()
  {
    auto sampled = std::make_shared<std::optional<frc::Pose2d>>();
    auto target = [this, sampled]
    {
      auto goal = Target();
      bool track = frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false);
      track = track || environment_->TrackTarget();
      if (goal && (!*sampled || track))
        *sampled = goal;
      return goal;
    };
    return frc2::cmd::Defer(
               [this, target]() -> frc2::CommandPtr
               {
                 if (frc::SmartDashboard::GetBoolean("AutoDrive/UseConfiguredFootprint", false))
                 {
                   if (!configuredFootprint_ || configuredFootprint_->drive.follow.maxSpeed > drive_.Config().maxSpeed ||
                       configuredFootprint_->drive.follow.maxOmega > drive_.Config().maxAngularRate)
                   {
                     frc::SmartDashboard::PutString("AutoDrive/ConfiguredFootprintStatus",
                                                    "missing-invalid-or-over-limit");
                     return frc2::cmd::RunOnce([this]
                                               { drive_.Stop(); }, {&drive_});
                   }
                   auto config = configuredFootprint_->drive;
                   bool track = frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false);
                   track = track || environment_->TrackTarget();
                   if (track)
                     config.targetTranslationReplanThreshold = autodriveconfig::kTargetReplanDistanceMetres;
                   const auto obstacles = configuredFootprint_->obstacles;
                   auto outcome = std::make_shared<base::FootprintAutoDriveOutcome>();
                   auto entry = nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/FootprintOutcome");
                   entry.SetString("running");
                   frc::SmartDashboard::PutString("AutoDrive/ConfiguredFootprintStatus", "ready");
                   return base::AutoDriveFootprintToPose(
                              drive_, target, [obstacles]
                              { return obstacles; }, config, outcome)
                       .FinallyDo([entry, outcome](bool) mutable
                                  { entry.SetString(FootprintOutcomeName(*outcome)); });
                 }
                 if (auto command = environment_->NavigationCommand(drive_, target))
                   return std::move(*command);
                 bool track = frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false);
                 track = track || environment_->TrackTarget();
                 if (track)
                   return base::AutoDriveTrackTarget(drive_, target);
                 return base::AutoDriveToPose(drive_, target);
               },
               {&drive_})
        .BeforeStarting(
            [this, sampled]
            {
              sampled->reset();
              nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(true);
              Status("planning-native");
            })
        // Retain the drivetrain until the stop has had time to settle; reporting
        // arrival immediately after the last commanded output can observe coasting.
        .AndThen(frc2::cmd::Wait(units::second_t{autodriveconfig::kPostCommandWaitSeconds}))
        .FinallyDo(
            [this, sampled](bool interrupted)
            {
              nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Active").SetBoolean(false);
              const auto &goal = *sampled;
              const bool reached = goal && AutoDriveArrived(drive_.Pose(), drive_.Speeds(), *goal);
              Status(interrupted ? "interrupted"
                     : reached   ? "arrived"
                     : goal      ? "stopped-before-goal"
                                 : "stopped-no-target");
            });
  }
  void AutoDrive::ClearPerception() { perception_->Clear(); }
} // namespace season
