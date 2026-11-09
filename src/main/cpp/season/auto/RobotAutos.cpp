#include "season/drive/DriveBehaviorConfig.h"
#include "season/auto/RobotAutos.h"
#include "season/RobotControlConfig.h"
#include "RobotContainer.h"
#include "season/runtime/RobotRuntime.h"
#include "season/runtime/SeasonPolicy.h"
#include "season/runtime/MatchContext.h"
#include <base/ReplaySettings.h>
#include "season/mechanisms/Mechanisms.h"
#include "season/vision/VisionConfig.h"
#include "base/ChoreoAuto.h"
#include <base/TelemetryPublishers.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <algorithm>
#include <memory>
#include <cmath>
#include <numbers>
#include <stdexcept>
using namespace units::literals;
void season::autonomous::RobotAutos::ConfigureDrive(base::Drive &drive)
{
  base::ConfigureNativeAutoDrivePathfinder();
  drive.ConfigureAuto();
}
void season::autonomous::RobotAutos::AddChooserOption(std::string label, std::string id,
                                                       frc2::CommandPtr command, bool isDefault)
{
  idsByLabel_[label] = id;
  commandsById_[id] = command.get();
  autos_.emplace_back(std::move(command));
  if (isDefault)
    chooser_.SetDefaultOption(label, id);
  else
    chooser_.AddOption(label, id);
}

frc2::Command *season::autonomous::RobotAutos::Selected()
{
  std::string selected;
  if (!base::replay::Active())
    selected = base::replay::ProcessValue<std::string>("Season/Autonomous/Selection", [this]
                                                       { return chooser_.GetSelected(); });
  else if (const auto recorded = base::replay::RecordedInputIfPresent("Season/Autonomous/Selection"))
  {
    selected = recorded->get<std::string>();
    base::replay::RecordOutput("EffectiveInputs/Season/Autonomous/Selection", *recorded);
  }
  else if (const auto label = base::replay::RecordedInputIfPresent("Auto/Selection"))
  {
    const auto found = idsByLabel_.find(label->get<std::string>());
    if (found == idsByLabel_.end())
      throw std::runtime_error("recorded autonomous chooser label is unavailable in this robot: " +
                               label->get<std::string>());
    selected = found->second;
    base::replay::RecordOutput("EffectiveInputs/Season/Autonomous/Selection", selected);
  }
  else
    throw std::runtime_error("recorded autonomous selection is unavailable in this log");
  if (selected.starts_with("PathPlanner/") && selected != "PathPlanner/None")
    base::replay::VerifyPathPlannerAssets();
  const auto command = commandsById_.find(selected);
  if (command != commandsById_.end())
    return command->second;
  if (base::replay::Active())
    throw std::runtime_error("recorded autonomous selection is unavailable in this deploy: " + selected);
  return nullptr;
}

void season::autonomous::RobotAutos::Configure()
{
  auto m = owner_.Mechanisms();
  // PathPlanner markers reuse commands with explicit mechanism requirements.
  pathplanner::NamedCommands::registerCommand(
      "DeployIntake", frc2::cmd::RunOnce(
                          [this] { owner_.rack_.SetPosition(season::mechanismconfig::kRackDeploy); },
                          {&owner_.rack_}));
  pathplanner::NamedCommands::registerCommand(
      "StowIntake", frc2::cmd::RunOnce(
                        [this] { owner_.rack_.SetPosition(season::mechanismconfig::kRackStow); },
                        {&owner_.rack_}));
  // Legacy name used by deployed autos; this is a coordinated shot, not just rollers.
  pathplanner::NamedCommands::registerCommand("FeedRollers", HubShot(false));
  pathplanner::NamedCommands::registerCommand("CoordinatedHubShot", HubShot(false));
  // Use this marker only in auto steps that do not run a path concurrently.
  pathplanner::NamedCommands::registerCommand("AimedHubShot", HubShot(true));
  pathplanner::NamedCommands::registerCommand(
      "IdleRollers", frc2::cmd::RunOnce(
                         [this]
                         {
                           owner_.conveyor_.Stop();
                           owner_.bottom_.Stop();
                           owner_.top_.Stop();
                         },
                         {&owner_.conveyor_, &owner_.bottom_, &owner_.top_}));
  pathplanner::NamedCommands::registerCommand(
      "Agitate",
      owner_.bottom_.VoltageCommand([] { return season::mechanismconfig::kBottomFeedVolts; })
          .AlongWith(owner_.top_.VoltageCommand([] { return season::mechanismconfig::kTopFeedVolts; })));
  pathplanner::NamedCommands::registerCommand(
      "SpinIntake", owner_.intake_.VoltageCommand([] { return season::mechanismconfig::kIntakeVolts; })
                        .AlongWith(owner_.conveyor_.VoltageCommand(
                            [] { return season::mechanismconfig::kAutoIntakeConveyorVolts; }))
                        .AlongWith(frc2::cmd::Run([this]
                        {
                          owner_.fuelEstimate_->Observe(owner_.autoDrive_->VisibleFuel(), owner_.drive_.Pose());
                          base::telemetry::PostDashboardNumber("AutoDrive/EstimatedFuel", owner_.fuelEstimate_->Count());
                        }))
                        .BeforeStarting([this] { owner_.fuelEstimate_->StartCollection(); }));
  // Spin up from the current pose while the auto is running.
  auto calculatedSpinUp = [this]
  {
    return frc2::cmd::RunEnd(
        [this]
        {
          auto p = owner_.drive_.Pose();
          auto alliance = owner_.MatchContext().alliance.value_or(frc::DriverStation::Alliance::kRed);
          auto shot = season::CalculateShot(p, season::HubPosition(alliance), season::runtime::SeasonPolicy().shot);
          owner_.hood_.SetPosition(shot.hoodRotations);
          owner_.shooter_.SetVelocity(shot.flywheelRps);
        },
        [this]
        {
          owner_.hood_.Stop();
          owner_.shooter_.Stop();
        },
        {&owner_.hood_, &owner_.shooter_});
  };
  pathplanner::NamedCommands::registerCommand(
      "AutoSpinUp", calculatedSpinUp().WithTimeout(units::second_t{config::kAutoSpinUpTimeoutSeconds}));
  pathplanner::NamedCommands::registerCommand(
      "Fender", season::ShooterPreset(m, season::shotconfig::kHubHood, season::shotconfig::kHubRps));
  // AutoAim owns the drivetrain while it rotates toward the selected target.
  pathplanner::NamedCommands::registerCommand(
      "AutoAim",
      frc2::cmd::RunEnd(
          [this]
          {
            auto p = owner_.drive_.Pose();
            auto target = SelectTarget();
            auto shot = season::CalculateShot(p, target, owner_.field_, season::runtime::SeasonPolicy().shot);
            owner_.hood_.SetPosition(shot.hoodRotations);
            owner_.shooter_.SetVelocity(shot.flywheelRps);
            double error = std::remainder(shot.headingRadians - p.Rotation().Radians().value(), 2 * std::numbers::pi);
            owner_.drive_.RunVelocity(
                frc::ChassisSpeeds{
                    0_mps, 0_mps,
                    units::radians_per_second_t{std::clamp(season::driveconfig::kAutoAimHeadingGain * error,
                                                          -season::driveconfig::kAutoAimMaxAngularRate,
                                                          season::driveconfig::kAutoAimMaxAngularRate)}});
          },
          [this]
          {
            owner_.drive_.Stop();
            owner_.hood_.Stop();
            owner_.shooter_.Stop();
          },
          {&owner_.drive_, &owner_.hood_, &owner_.shooter_}));
  // Combine deployed PathPlanner autos with season-specific options.
  for (const auto &name : pathplanner::AutoBuilder::getAllAutoNames())
    AddChooserOption(name, "PathPlanner/" + name,
                     frc2::CommandPtr{std::make_unique<pathplanner::PathPlannerAuto>(name)});
  AddChooserOption("None", "PathPlanner/None", frc2::cmd::None(), true);

  const auto defaults = season::runtime::SeasonPolicy().autonomous;
  AddChooserOption("Hub shot (vision required)", "Season/HubShotVisionRequired", HubShot(true, true),
                   !defaults.offensiveDefault);
  AddChooserOption("Choreo LeftBlueBump + Shoot", "Season/ChoreoLeftBlueBumpShoot",
                   base::FollowChoreo<2026>(owner_.drive_, "LeftBlueBump", {config::kChoreoTranslationP, 0, 0},
                                            {config::kChoreoRotationP, 0, 0}, season::hardware::kControlPeriod)
                       .AndThen(HubShot()));
  AddChooserOption("Do nothing", "Season/DoNothing", frc2::cmd::None());
  AddChooserOption("AutoDrive target", "Season/AutoDriveTarget", owner_.autoDrive_->Command());
  AddChooserOption("AutoDrive offensive", "Season/AutoDriveOffensive",
                   season::OffensiveAutoDrive(*owner_.autoDrive_, owner_.drive_, m, owner_.field_, owner_.fuelEstimate_),
                   defaults.offensiveDefault);
  frc::SmartDashboard::PutBoolean("AutoDrive/OffenseEnabled", defaults.offenseEnabled);
  frc::SmartDashboard::PutData("Auto Choices", &chooser_);
  if (auto practice = owner_.runtime_->PracticeMatchCommand())
  {
    autos_.emplace_back(std::move(*practice));
    frc::SmartDashboard::PutData("Start Practice Match", autos_.back().get());
  }
}
// Select a hub or ferry target from alliance, pose, and visible tags.
season::ShotTarget season::autonomous::RobotAutos::SelectTarget()
{
  auto alliance = owner_.MatchContext().alliance.value_or(frc::DriverStation::Alliance::kRed);
  auto calibration = base::replay::NetworkBoolean("Season/NT//TunableNumbers/ShooterCalibration/Enabled", "/TunableNumbers/ShooterCalibration/Enabled", false);
  if (calibration)
    return {season::ShotMode::Hub, season::HubPosition(alliance)};
  auto p = owner_.drive_.Pose();
  std::vector<int> ids;
  if (const auto *tags = owner_.runtime_->TagIds())
    return season::SelectTarget(alliance, p, *tags, owner_.field_);
  // Limelight publishes each raw fiducial as a seven-value record.
  auto raw = base::replay::NetworkNumberArray("Season/NT//limelight/rawfiducials", "/limelight/rawfiducials");
  if (owner_.vision_->Inputs().connected)
    for (size_t i = 0; i + 6 < raw.size(); i += 7)
      ids.push_back(static_cast<int>(raw[i]));
  return season::SelectTarget(alliance, p, ids, owner_.field_);
}
// Build a bounded shot with optional drivetrain ownership and vision gating.
frc2::CommandPtr season::autonomous::RobotAutos::HubShot(bool ownDrive, bool requireVision)
{
  season::CoordinatedShotOptions o;
  o.ownDrive = ownDrive;
  o.autonomous = true;
  o.shotCalibration = season::runtime::SeasonPolicy().shot;
  // A shot ends as soon as its configured feed completes. Its overall bound
  // must also allow the rollers to reverse after the preceding intake path.
  o.readyTimeoutSeconds = season::shotconfig::kShotReadyTimeoutSeconds;
  o.totalTimeoutSeconds = season::runtime::SeasonPolicy().behavior.shotTimeoutSeconds;
  o.feedDurationSeconds = season::ConfiguredFeedDurationSeconds;
  struct FeedTiming
  {
    double last = 0, elapsed = 0, target = 0;
    bool active = false;
  };
  auto timing = std::make_shared<FeedTiming>();
  o.feedFeedback = [timing](bool active)
  {
    const double now = frc::Timer::GetFPGATimestamp().value();
    if (timing->active)
      timing->elapsed += std::max(0., now - timing->last);
    timing->last = now;
    timing->active = active;
  };
  o.poseReady = [this, requireVision]
  {
    return (!requireVision && (owner_.MatchContext().enabled && owner_.MatchContext().autonomous)) ||
           frc::Timer::GetFPGATimestamp().value() - owner_.lastAcceptedVision_ <
               season::visionconfig::kVisionFreshnessSeconds;
  };
  return season::CoordinatedShot(
             owner_.Mechanisms(), owner_.drive_, owner_.field_,
             [this]
             {
               auto a = owner_.MatchContext().alliance.value_or(frc::DriverStation::Alliance::kRed);
               return season::ShotTarget{season::ShotMode::Hub, season::HubPosition(a)};
             },
             o)
      .BeforeStarting([timing]
                      {
                        timing->last = frc::Timer::GetFPGATimestamp().value();
                        timing->elapsed = 0;
                        timing->active = false;
                        timing->target = season::ConfiguredFeedDurationSeconds();
                      })
      .FinallyDo([this, timing](bool interrupted)
                 {
                   const bool complete = !interrupted && timing->target > 0 && timing->elapsed >= timing->target;
                   owner_.fuelEstimate_->CompleteFeed(complete);
                   base::replay::RecordOutput("Season/Shooter/FeedResult", wpi::json{{"completed", complete},
                                                                                     {"interrupted", interrupted},
                                                                                     {"elapsedSeconds", timing->elapsed},
                                                                                     {"targetSeconds", timing->target}});
                 });
}
