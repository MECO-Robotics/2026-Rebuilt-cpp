#include "season/drive/AutoDrive.h"
#include "season/RobotGeometry.h"
#include "season/runtime/MatchContext.h"
#include <base/ReplaySettings.h>
#include <base/TelemetryPublishers.h>
#include <frc2/command/Commands.h>
#include <frc/Preferences.h>
#include <frc/Filesystem.h>
#include <base/FootprintAutoDrive.h>
#include <base/VisionIOSelector.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
      std::size_t, const std::string &, std::function<frc::Transform3d()>, units::second_t)
  {
    return std::make_unique<base::InjectedObjectDetectionIO>([] { return std::nullopt; });
  }

  // Publish a concise reason for the current navigation state.
  void AutoDrive::Status(const char *value)
  {
    base::telemetry::PostNetworkString("/AutoDrive/Status", value);
    base::replay::RecordOutput("Season/AutoDrive/Status", value);
  }
  // Account for the front camera moving with the intake.
  frc::Transform3d AutoDrive::Mount(size_t camera) const
  {
    const std::string prefix = camera == 0 ? "Perception/Front/" : "Perception/Rear/";
    const auto &defaults = camera == 0 ? vision::config::kFrontCamera : vision::config::kRearCamera;
    // AutoDrive's nominal fore/aft chassis-edge locations; height is an estimate.
    const frc::Transform3d local{
        units::meter_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "X", prefix + "X", defaults.xMetres)},
        units::meter_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "Y", prefix + "Y", defaults.yMetres)},
        units::meter_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "Z", prefix + "Z", defaults.zMetres)},
        frc::Rotation3d{units::degree_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "RollDeg", prefix + "RollDeg", defaults.rollDegrees)},
                        units::degree_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "PitchDeg", prefix + "PitchDeg", defaults.pitchDegrees)},
                        units::degree_t{base::replay::PreferenceNumber(std::string{"Season/Preference/"} + prefix + "YawDeg", prefix + "YawDeg", defaults.yawDegrees)}}};
    return camera == 0 ? HopperCameraMount(extension_(), local) : local;
  }
  AutoDrive::AutoDrive(base::Drive &drive, std::function<double()> extension,
                       std::function<const season::runtime::MatchContext &()> context,
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
      auto io = base::MakeRuntimeObjectDetectionIO(
          "/Perception/" + name + "/detections", detectionUpdatePeriod,
          [this, i, name, detectionUpdatePeriod] {
            return environment_->Camera(i, name, [this, i] { return Mount(i); }, detectionUpdatePeriod);
          });
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
    base::telemetry::PostNetworkBoolean("/AutoDrive/Active", false);
    Status("idle-native");
  }
  void AutoDrive::LoadConfiguredFootprint()
  {
    if (configuredFootprintLoaded_)
      return;
    configuredFootprintLoaded_ = true;
    const std::string path = frc::filesystem::GetDeployDirectory() + "/autodrive-footprint.json";
    const auto contents = base::replay::ProcessFileInput(
        "Season/AutoDrive/FootprintConfiguration", path, autodriveconfig::kMaxFootprintConfigBytes);
    if (contents)
      configuredFootprint_ = base::ParseFootprintConfiguration(*contents);
    base::telemetry::PostDashboardString("AutoDrive/ConfiguredFootprintStatus",
                                         configuredFootprint_ ? "loaded" : "missing-or-invalid");
  }
  AutoDrive::~AutoDrive() = default;
  void AutoDrive::PublishMatchContext() const
  {
    const auto &match = MatchState();
    base::telemetry::PostDashboardString("AutoDrive/MatchPhase", match.phase);
    base::telemetry::PostDashboardNumber("AutoDrive/MatchTime", match.periodTime.value_or(-1));
    base::telemetry::PostDashboardBoolean("AutoDrive/HubKnown", match.ownActive.has_value());
    base::telemetry::PostDashboardBoolean("AutoDrive/HubActive", match.ownActive.value_or(false));
    base::telemetry::PostDashboardBoolean("AutoDrive/HubOrderKnown", match.gameDataValid);
    base::telemetry::PostDashboardString("AutoDrive/InactiveFirstAlliance",
                                   season::runtime::AllianceLabel(match.inactiveFirstAlliance));
    base::telemetry::PostDashboardString("AutoDrive/FirstActiveAlliance",
                                   season::runtime::AllianceLabel(match.firstActiveAlliance));
    base::telemetry::PostDashboardBoolean("AutoDrive/HubNextChangeKnown", match.ownNextChange.has_value());
    base::telemetry::PostDashboardNumber("AutoDrive/HubNextChangeSeconds", match.ownNextChange.value_or(-1));
  }
  // Return the perception snapshot, not simulation ground truth.
  const std::vector<base::ObjectCluster> &AutoDrive::VisibleFuel() const { return perception_->Clusters(); }
  // Follow visible fuel or use the fixed blue-origin dashboard target.
  std::optional<frc::Pose2d> AutoDrive::Target()
  {
    if (base::replay::DashboardBoolean("Season/Setting/AutoDrive/FollowFuel", "AutoDrive/FollowFuel", false))
    {
      const auto &clusters = perception_->Clusters();
      base::telemetry::PostDashboardBoolean("AutoDrive/DetectionFresh", !clusters.empty());
      if (clusters.empty())
      {
        Status("no-visible-fuel");
        base::replay::RecordOutput("Season/AutoDrive/Target", wpi::json{{"source", "visible-fuel"}, {"valid", false}});
        return std::nullopt;
      }
      const auto nearest = std::min_element(clusters.begin(), clusters.end(),
                                            [this](const auto &a, const auto &b)
                                            {
                                              return a.position.Distance(drive_.Pose().Translation()) <
                                                     b.position.Distance(drive_.Pose().Translation());
                                            });
      base::telemetry::PostDashboardNumber("AutoDrive/SelectedFuelX", nearest->position.X().value());
      base::telemetry::PostDashboardNumber("AutoDrive/SelectedFuelY", nearest->position.Y().value());
      base::replay::RecordOutput("Season/AutoDrive/Target", wpi::json{{"source", "visible-fuel"},
                                                                    {"valid", true},
                                                                    {"xMetres", nearest->position.X().value()},
                                                                    {"yMetres", nearest->position.Y().value()}});
      return frc::Pose2d{nearest->position, (nearest->position - drive_.Pose().Translation()).Angle()};
    }
    const double x = base::replay::DashboardNumber("Season/Setting/AutoDrive/TargetX", "AutoDrive/TargetX", autodriveconfig::kDefaultTargetXMetres);
    const double y = base::replay::DashboardNumber("Season/Setting/AutoDrive/TargetY", "AutoDrive/TargetY", autodriveconfig::kDefaultTargetYMetres);
    const double heading = base::replay::DashboardNumber("Season/Setting/AutoDrive/TargetHeading", "AutoDrive/TargetHeading", 0);
    base::replay::RecordOutput("Season/AutoDrive/Target", wpi::json{{"source", "configured"}, {"valid", true},
                                                                  {"xMetres", x}, {"yMetres", y}, {"headingRadians", heading}});
    return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::radian_t{heading}}};
  }
  // Select the configured navigation mode when the command starts.
  frc2::CommandPtr AutoDrive::Command()
  {
    auto sampled = std::make_shared<std::optional<frc::Pose2d>>();
    auto target = [this, sampled]
    {
      auto goal = Target();
      bool track = base::replay::DashboardBoolean("Season/Setting/AutoDrive/FollowFuel", "AutoDrive/FollowFuel", false);
      track = track || environment_->TrackTarget();
      if (goal && (!*sampled || track))
        *sampled = goal;
      return goal;
    };
    return frc2::cmd::Defer(
               [this, target]() -> frc2::CommandPtr
               {
                 LoadConfiguredFootprint();
                 if (base::replay::DashboardBoolean("Season/Setting/AutoDrive/UseConfiguredFootprint", "AutoDrive/UseConfiguredFootprint", false))
                 {
                   if (!configuredFootprint_ || configuredFootprint_->drive.follow.maxSpeed > drive_.Config().maxSpeed ||
                       configuredFootprint_->drive.follow.maxOmega > drive_.Config().maxAngularRate)
                   {
                     base::telemetry::PostDashboardString("AutoDrive/ConfiguredFootprintStatus",
                                                    "missing-invalid-or-over-limit");
                     return frc2::cmd::RunOnce([this]
                                               { drive_.Stop(); }, {&drive_});
                   }
                   auto config = configuredFootprint_->drive;
                   bool track = base::replay::DashboardBoolean("Season/Setting/AutoDrive/FollowFuel", "AutoDrive/FollowFuel", false);
                   track = track || environment_->TrackTarget();
                   if (track)
                     config.targetTranslationReplanThreshold = autodriveconfig::kTargetReplanDistanceMetres;
                   const auto obstacles = configuredFootprint_->obstacles;
                   auto outcome = std::make_shared<base::FootprintAutoDriveOutcome>();
                   base::telemetry::PostNetworkString("/AutoDrive/FootprintOutcome", "running");
                   base::telemetry::PostDashboardString("AutoDrive/ConfiguredFootprintStatus", "ready");
                   return base::AutoDriveFootprintToPose(
                              drive_, target, [obstacles]
                              { return obstacles; }, config, outcome)
                       .FinallyDo([outcome](bool) mutable
                                  { base::telemetry::PostNetworkString("/AutoDrive/FootprintOutcome", base::FootprintOutcomeName(*outcome)); });
                 }
                 if (auto command = environment_->NavigationCommand(drive_, target))
                   return std::move(*command);
                 bool track = base::replay::DashboardBoolean("Season/Setting/AutoDrive/FollowFuel", "AutoDrive/FollowFuel", false);
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
              base::telemetry::PostNetworkBoolean("/AutoDrive/Active", true);
              Status("planning-native");
            })
        // Retain the drivetrain until the stop has had time to settle; reporting
        // arrival immediately after the last commanded output can observe coasting.
        .AndThen(frc2::cmd::Wait(units::second_t{autodriveconfig::kPostCommandWaitSeconds}))
        .FinallyDo(
            [this, sampled](bool interrupted)
            {
              base::telemetry::PostNetworkBoolean("/AutoDrive/Active", false);
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
