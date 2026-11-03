#include "season/auto/RobotAutos.h"
#include "season/drive/AutoDriveConfig.h"
#include "RobotContainer.h"
#include "season/simulation/RobotRuntime.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/MechanismConfig.h"
#include "base/ChoreoAuto.h"
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#ifndef __FRC_ROBORIO__
#include <frc/simulation/DriverStationSim.h>
#endif
#include <algorithm>
#include <cmath>
#include <numbers>
using namespace units::literals;
void season::autonomous::RobotAutos::ConfigureDrive(base::Drive &drive) {
  base::ConfigureNativeAutoDrivePathfinder();
  drive.ConfigureAuto();
}
void season::autonomous::RobotAutos::Configure() {
  auto m = owner_.Mechanisms();
  // PathPlanner markers reuse commands with explicit mechanism requirements.
  pathplanner::NamedCommands::registerCommand("DeployIntake",
                                              frc2::cmd::RunOnce([this]
                                                                 { owner_.rack_.SetPosition(season::hardware::kRackDeploy); }, {&owner_.rack_}));
  pathplanner::NamedCommands::registerCommand("StowIntake",
                                              frc2::cmd::RunOnce([this]
                                                                 { owner_.rack_.SetPosition(season::hardware::kRackStow); }, {&owner_.rack_}));
  pathplanner::NamedCommands::registerCommand("FeedRollers", HubShot(false));
  pathplanner::NamedCommands::registerCommand("CoordinatedHubShot", HubShot(false));
  // Use this marker only in auto steps that do not run a path concurrently.
  pathplanner::NamedCommands::registerCommand("AimedHubShot", HubShot(true));
  pathplanner::NamedCommands::registerCommand("IdleRollers", frc2::cmd::RunOnce(
                                                                 [this]
                                                                 {
                                                                   owner_.conveyor_.Stop();
                                                                   owner_.bottom_.Stop();
                                                                   owner_.top_.Stop();
                                                                 },
                                                                 {&owner_.conveyor_, &owner_.bottom_, &owner_.top_}));
  pathplanner::NamedCommands::registerCommand("Agitate", frc2::cmd::RunEnd(
                                                             [this]
                                                             {
                                                               owner_.bottom_.SetVoltage(season::hardware::kBottomFeedVolts);
                                                               owner_.top_.SetVoltage(season::hardware::kTopFeedVolts);
                                                             },
                                                             [this]
                                                             {
                                                               owner_.bottom_.Stop();
                                                               owner_.top_.Stop();
                                                             },
                                                             {&owner_.bottom_, &owner_.top_}));
  pathplanner::NamedCommands::registerCommand("SpinIntake", frc2::cmd::RunEnd(
                                                                [this]
                                                                {
                                                                  owner_.intake_.SetVoltage(season::hardware::kIntakeVolts);
                                                                  owner_.conveyor_.SetVoltage(season::hardware::kAutoIntakeConveyorVolts);
                                                                  owner_.fuelEstimate_->Observe(owner_.autoDrive_->VisibleFuel(), owner_.drive_.Pose());
                                                                  frc::SmartDashboard::PutNumber("AutoDrive/EstimatedFuel", owner_.fuelEstimate_->Count());
                                                                },
                                                                [this]
                                                                {
                                                                  owner_.intake_.Stop();
                                                                  owner_.conveyor_.Stop();
                                                                },
                                                                {&owner_.intake_, &owner_.conveyor_})
                                                                .BeforeStarting([this]
                                                                                { owner_.fuelEstimate_->StartCollection(); }));
  // Spin up from the current pose while the auto is running.
  auto calculatedSpinUp = [this]
  {
    return frc2::cmd::RunEnd(
        [this]
        {
          auto p = owner_.drive_.Pose();
          auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
          auto shot = season::CalculateShot(p, season::HubPosition(alliance), frc::RobotBase::IsSimulation());
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
  pathplanner::NamedCommands::registerCommand("AutoSpinUp", calculatedSpinUp().WithTimeout(units::second_t{season::autodriveconfig::kAutoSpinUpTimeoutSeconds}));
  pathplanner::NamedCommands::registerCommand("Fender", season::ShooterPreset(m, season::hardware::kHubHood, season::hardware::kHubRps));
  // AutoAim owns the drivetrain while it rotates toward the selected target.
  pathplanner::NamedCommands::registerCommand(
      "AutoAim",
      frc2::cmd::RunEnd(
          [this]
          {
            auto p = owner_.drive_.Pose();
            auto target = SelectTarget();
            auto shot = season::CalculateShot(p, target, owner_.field_, frc::RobotBase::IsSimulation());
            owner_.hood_.SetPosition(shot.hoodRotations);
            owner_.shooter_.SetVelocity(shot.flywheelRps);
            double error = std::remainder(shot.headingRadians - p.Rotation().Radians().value(), 2 * std::numbers::pi);
            owner_.drive_.RunVelocity(
                frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{std::clamp(season::hardware::kAutoAimHeadingGain * error, -season::hardware::kAutoAimMaxAngularRate, season::hardware::kAutoAimMaxAngularRate)}});
          },
          [this]
          {
            owner_.drive_.Stop();
            owner_.hood_.Stop();
            owner_.shooter_.Stop();
          },
          {&owner_.drive_, &owner_.hood_, &owner_.shooter_}));
  // Combine deployed PathPlanner autos with season-specific options.
  chooser_ = pathplanner::AutoBuilder::buildAutoChooser();
  autos_.emplace_back(HubShot(true, true));
  autos_.emplace_back(
      base::FollowChoreo<2026>(owner_.drive_, "LeftBlueBump", {season::autodriveconfig::kChoreoTranslationP, 0, 0}, {season::autodriveconfig::kChoreoRotationP, 0, 0}, season::hardware::kControlPeriod).AndThen(HubShot()));
  autos_.emplace_back(frc2::cmd::None());
  // Hardware and desktop simulation intentionally choose different defaults.
  if (!frc::RobotBase::IsSimulation())
    chooser_.SetDefaultOption("Fender I HARDLY KNOW HER -JAVI", autos_[0].get());
  else
    chooser_.AddOption("Fender I HARDLY KNOW HER -JAVI", autos_[0].get());
  chooser_.AddOption("Choreo LeftBlueBump + Shoot", autos_[1].get());
  chooser_.AddOption("You better hit the A stop before this -Manny (none)", autos_[2].get());
  autos_.emplace_back(owner_.autoDrive_->Command());
  chooser_.AddOption("AutoDrive target", autos_.back().get());
  autos_.emplace_back(season::OffensiveAutoDrive(*owner_.autoDrive_, owner_.drive_, m, owner_.field_, owner_.fuelEstimate_));
  if (frc::RobotBase::IsSimulation())
    chooser_.SetDefaultOption("AutoDrive offensive", autos_.back().get());
  else
    chooser_.AddOption("AutoDrive offensive", autos_.back().get());
  frc::SmartDashboard::PutBoolean("AutoDrive/OffenseEnabled", frc::RobotBase::IsSimulation());
  frc::SmartDashboard::PutData("Auto Choices", &chooser_);
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation())
  {
    // The dashboard command starts practice using the selected sim station.
    autos_.emplace_back(frc2::cmd::RunOnce(
                                  []
                                  {
                                    const char *station = "Blue1";
                                    switch (frc::sim::DriverStationSim::GetAllianceStationId())
                                    {
                                    case HAL_AllianceStationID_kRed1:
                                      station = "Red1";
                                      break;
                                    case HAL_AllianceStationID_kRed2:
                                      station = "Red2";
                                      break;
                                    case HAL_AllianceStationID_kRed3:
                                      station = "Red3";
                                      break;
                                    case HAL_AllianceStationID_kBlue2:
                                      station = "Blue2";
                                      break;
                                    case HAL_AllianceStationID_kBlue3:
                                      station = "Blue3";
                                      break;
                                    default:
                                      break;
                                    }
                                    nt::NetworkTableInstance::GetDefault()
                                        .GetTable("/Sim/Practice")
                                        ->GetEntry("Config/AllianceStation")
                                        .SetString(station);
                                    static auto command = nt::NetworkTableInstance::GetDefault()
                                                              .GetStringTopic("/Sim/Practice/Command")
                                                              .Publish({.keepDuplicates = true});
                                    command.Set("start");
                                  })
                                  .IgnoringDisable(true));
    frc::SmartDashboard::PutData("Start Practice Match", autos_.back().get());
  }
#endif
}
// Select a hub or ferry target from alliance, pose, and visible tags.
season::ShotTarget season::autonomous::RobotAutos::SelectTarget()
{
  auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
  auto calibration =
      nt::NetworkTableInstance::GetDefault().GetEntry("/TunableNumbers/ShooterCalibration/Enabled").GetBoolean(false);
  if (calibration)
    return {season::ShotMode::Hub, season::HubPosition(alliance)};
  auto p = owner_.drive_.Pose();
  std::vector<int> ids;
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  if (const auto *tags = owner_.simulationRuntime_->TagIds())
    return season::SelectTarget(alliance, p, *tags, owner_.field_);
  // Limelight publishes each raw fiducial as a seven-value record.
  auto raw = table->GetNumberArray("rawfiducials", {});
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
  o.simulation = frc::RobotBase::IsSimulation();
  // A shot ends as soon as its configured feed completes. Its overall bound
  // must also allow the rollers to reverse after the preceding intake path.
  o.readyTimeoutSeconds = season::hardware::kShotReadyTimeoutSeconds;
  o.totalTimeoutSeconds = o.simulation ? season::hardware::kShotSimTimeoutSeconds : season::hardware::kShotRealTimeoutSeconds;
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
    return (!requireVision && frc::DriverStation::IsAutonomousEnabled()) ||
           frc::Timer::GetFPGATimestamp().value() - owner_.lastAcceptedVision_ < season::hardware::kVisionFreshnessSeconds;
  };
  return season::CoordinatedShot(
             owner_.Mechanisms(), owner_.drive_, owner_.field_,
             []
             {
               auto a = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
               return season::ShotTarget{season::ShotMode::Hub, season::HubPosition(a)};
             },
             o)
      .BeforeStarting([timing]
                      {
        timing->last = frc::Timer::GetFPGATimestamp().value();
        timing->elapsed = 0;
        timing->active = false;
        timing->target = season::ConfiguredFeedDurationSeconds(); })
      .FinallyDo([this, timing](bool interrupted)
                 { owner_.fuelEstimate_->CompleteFeed(!interrupted && timing->target > 0 && timing->elapsed >= timing->target); });
}
