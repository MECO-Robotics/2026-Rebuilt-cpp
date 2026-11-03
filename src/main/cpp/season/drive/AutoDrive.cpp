#include "season/drive/AutoDrive.h"
#include "season/simulation/RuntimeConfig.h"
#include "season/mechanisms/RobotGeometry.h"
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <frc/Timer.h>
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
#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include <meco/sim/SimulatedPieceSensor.h>
#endif

namespace season
{
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
#ifndef __FRC_ROBORIO__
  struct AutoDrive::SimulationSensors
  {
    std::array<std::unique_ptr<meco::sim::SimulatedPieceSensor>, 2> sensors;
    std::array<std::optional<base::ObjectDetectionFrame>, 2> frames;
    uint64_t sequence{};
  };
#endif
  // Publish a concise reason for the current navigation state.
  void AutoDrive::Status(const char *value)
  {
    nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/Status").SetString(value);
  }
  // Account for the front camera moving with the intake.
  frc::Transform3d AutoDrive::Mount(size_t camera) const
  {
    const std::string prefix = camera == 0 ? "Perception/Front/" : "Perception/Rear/";
    // AutoDrive's nominal fore/aft chassis-edge locations; height is an estimate.
    const frc::Transform3d local{
        units::meter_t{frc::Preferences::GetDouble(prefix + "X", camera == 0 ? autodriveconfig::kCameraFrontXMetres : autodriveconfig::kCameraRearXMetres)},
        units::meter_t{frc::Preferences::GetDouble(prefix + "Y", 0)},
        units::meter_t{frc::Preferences::GetDouble(prefix + "Z", autodriveconfig::kCameraHeightMetres)},
        frc::Rotation3d{units::degree_t{frc::Preferences::GetDouble(prefix + "RollDeg", 0)},
                        units::degree_t{frc::Preferences::GetDouble(prefix + "PitchDeg", autodriveconfig::kCameraPitchDegrees)},
                        units::degree_t{frc::Preferences::GetDouble(prefix + "YawDeg", camera == 0 ? 0 : autodriveconfig::kRearCameraYawDegrees)}}};
    return camera == 0 ? HopperCameraMount(extension_(), local) : local;
  }
  AutoDrive::AutoDrive(base::Drive &drive, std::function<double()> extension,
                       std::function<const base::season2026::MatchContext &()> context,
                       units::second_t detectionUpdatePeriod)
      : matchContext_(std::move(context)), drive_(drive), extension_(std::move(extension))
  {
    std::vector<base::ObjectCamera> cameras;
#ifndef __FRC_ROBORIO__
    simulationSensors_ = std::make_unique<SimulationSensors>();
#endif
    for (size_t i = 0; i < 2; ++i)
    {
      const std::string name = i == 0 ? "front" : "rear";
      const std::string prefix = i == 0 ? "Perception/Front/" : "Perception/Rear/";
      for (const auto &[key, value] : std::vector<std::pair<std::string, double>>{{"X", i == 0 ? autodriveconfig::kCameraFrontXMetres : autodriveconfig::kCameraRearXMetres},
                                                                                  {"Y", 0},
                                                                                  {"Z", autodriveconfig::kCameraHeightMetres},
                                                                                  {"RollDeg", 0},
                                                                                  {"PitchDeg", autodriveconfig::kCameraPitchDegrees},
                                                                                  {"YawDeg", i == 0 ? 0 : autodriveconfig::kRearCameraYawDegrees}})
        frc::Preferences::InitDouble(prefix + key, value);
      std::unique_ptr<base::ObjectDetectionIO> io;
#ifndef __FRC_ROBORIO__
      if (frc::RobotBase::IsSimulation())
      {
        meco::sim::PieceSensorConfig config;
        config.maxRange = autodriveconfig::kCameraRangeMetres;
        config.seed = simulation::runtimeconfig::kFuelRandomSeed + i;
        config.robotToCamera = Mount(i);
        simulationSensors_->sensors[i] = std::make_unique<meco::sim::SimulatedPieceSensor>(config);
        io = std::make_unique<base::InjectedObjectDetectionIO>([this, i]
                                                               { return std::exchange(simulationSensors_->frames[i], std::nullopt); });
      }
      else
#endif
      {
        io = base::MakeNtObjectDetectionIO("/Perception/" + name + "/detections", detectionUpdatePeriod);
      }
      cameras.push_back({name, std::move(io), [this, i]() -> std::optional<frc::Transform3d>
                         { return Mount(i); }});
    }
    base::ObjectPerceptionConfig perceptionConfig;
    // Fuse duplicate camera observations without chaining adjacent FUEL into
    // one field-wide centroid (the simulation places centres 0.25 m apart).
    perceptionConfig.clusterRadius = units::meter_t{autodriveconfig::kClusterRadiusMetres};
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
#ifndef __FRC_ROBORIO__
    frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintSim", false);
    frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintProfileSim", false);
    frc::SmartDashboard::PutBoolean("AutoDrive/AvoidBumpsSim", false);
    frc::SmartDashboard::PutBoolean("AutoDrive/TrackTargetSim", false);
#endif
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
#ifndef __FRC_ROBORIO__
      track = track || frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false);
#endif
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
#ifndef __FRC_ROBORIO__
                   track = track || frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false);
#endif
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
#ifndef __FRC_ROBORIO__
                 if (frc::RobotBase::IsSimulation() &&
                     frc::SmartDashboard::GetBoolean("AutoDrive/UseFootprintSim", false))
                 {
                   base::FootprintAutoDriveConfig config;
                   const bool avoidBumps = frc::SmartDashboard::GetBoolean("AutoDrive/AvoidBumpsSim", false);
                   config.footprint = {autodriveconfig::kFootprintLengthMetres, autodriveconfig::kFootprintWidthMetres};
                   config.field = {0, 0, simulation::kFieldLength, simulation::kFieldWidth};
                   if (frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false) ||
                       frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false))
                     config.targetTranslationReplanThreshold = autodriveconfig::kTargetReplanDistanceMetres;
                   // The modeled swerve can drift after a rotation if it releases the
                   // drivetrain while residual angular speed is still appreciable.
                   config.follow.headingTolerance = autodriveconfig::kHeadingToleranceRadians;
                   config.follow.angularSettleSpeed = autodriveconfig::kAngularSettleSpeed;
                   config.follow.settleSamples = 10;
                   config.follow.finalSettleSamples = autodriveconfig::kFinalSettleSamples;
                   config.follow.poseAngularDamping = autodriveconfig::kPoseAngularDamping;
                   config.follow.usePoseRateForSettling = true;
                   if (frc::SmartDashboard::GetBoolean("AutoDrive/UseFootprintProfileSim", false))
                   {
                     config.follow.terrain = simulation::RampTerrain;
                     config.follow.terrainBoundaries = simulation::RampBoundaries;
                     config.follow.angularStopTolerance = autodriveconfig::kAngularStopTolerance;
                     config.follow.translationStallTimeout = autodriveconfig::kTranslationStallTimeoutSeconds;
                     config.follow.translationStallRequest = autodriveconfig::kTranslationStallRequest;
                     // Illustrative simulation-only capability, not a measured robot limit.
                     config.follow.maxAcceleration = autodriveconfig::kMaxAcceleration;
                   }
                   auto obstacles = [avoidBumps]
                   {
                     std::vector<base::FieldRectangle> boxes;
                     if (frc::SmartDashboard::GetBoolean("FieldSimulation/CollidersEnabled", true))
                       for (const auto &body : simulation::FieldColliders())
                       {
                         // Skip traversable inclined ramps and objects entirely above or
                         // below the modeled chassis contact volume. Remaining field
                         // boxes are axis aligned in this desktop model.
                         if (std::abs(body.pose.rotation.x) > 1e-8 || std::abs(body.pose.rotation.y) > 1e-8 ||
                             std::abs(body.pose.rotation.z) > 1e-8 || body.pose.position.z + body.halfExtents.z < autodriveconfig::kCollisionBottomThresholdMetres ||
                             body.pose.position.z - body.halfExtents.z > autodriveconfig::kCollisionTopThresholdMetres)
                           continue;
                         boxes.push_back(
                             {body.pose.position.x - body.halfExtents.x, body.pose.position.y - body.halfExtents.y,
                              body.pose.position.x + body.halfExtents.x, body.pose.position.y + body.halfExtents.y});
                       }
                     if (frc::SmartDashboard::GetBoolean("FieldSimulation/CollidersEnabled", true) && avoidBumps)
                     {
                       const auto ramps = simulation::RampObstacles();
                       boxes.insert(boxes.end(), ramps.begin(), ramps.end());
                     }
                     // Desktop HAL/NT tests can inject one perceived obstacle. It is
                     // not a simulated rigid body or a hardware detection source.
                     const auto observed = nt::NetworkTableInstance::GetDefault()
                                               .GetEntry("/Simulation/FootprintObstacle")
                                               .GetDoubleArray({});
                     if (!observed.empty())
                     {
                       if (observed.size() != 4)
                         throw std::runtime_error("invalid simulated footprint obstacle");
                       boxes.push_back({observed[0], observed[1], observed[2], observed[3]});
                     }
                     return boxes;
                   };
                   auto outcome = std::make_shared<base::FootprintAutoDriveOutcome>();
                   auto entry = nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/FootprintOutcome");
                   entry.SetString("running");
                   return base::AutoDriveFootprintToPose(drive_, target, std::move(obstacles), config, outcome)
                       .FinallyDo([entry, outcome](bool) mutable
                                  { entry.SetString(FootprintOutcomeName(*outcome)); });
                 }
#endif
                 bool track = frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false);
#ifndef __FRC_ROBORIO__
                 track = track || frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false);
#endif
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
#ifndef __FRC_ROBORIO__
  // Feed synthetic camera frames through the same perception interface.
  void AutoDrive::UpdateSimulation(const frc::Pose3d &truth, const FuelModel &fuel, bool fieldColliders)
  {
    std::vector<frc::Translation3d> points;
    points.reserve(fuel.FloorFuel().size());
    for (const auto &p : fuel.FloorFuel())
      points.emplace_back(units::meter_t{p.position.x}, units::meter_t{p.position.y}, units::meter_t{p.position.z});
    static const auto fieldObstacles = simulation::FieldColliders();
    static const std::vector<base::sim::BodySpec> noObstacles;
    const auto &obstacles = fieldColliders ? fieldObstacles : noObstacles;
    ++simulationSensors_->sequence;
    for (size_t i = 0; i < 2; ++i)
    {
      simulationSensors_->sensors[i]->SetMount(Mount(i));
      simulationSensors_->frames[i] = simulationSensors_->sensors[i]->ObserveFrame(
          truth, points, obstacles, frc::Timer::GetFPGATimestamp(), simulationSensors_->sequence);
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontDetections" : "AutoDrive/SimRearDetections",
                                     simulationSensors_->frames[i]->detections.size());
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontFrameAge" : "AutoDrive/SimRearFrameAge",
                                     (frc::Timer::GetFPGATimestamp() - simulationSensors_->frames[i]->captureTime).value());
    }
    const auto &clusters = perception_->Clusters();
    frc::SmartDashboard::PutBoolean("AutoDrive/DetectionFresh", !clusters.empty());
    frc::SmartDashboard::PutNumber("AutoDrive/ClusterCount", clusters.size());
    double nearest = -1;
    const auto robotPosition = drive_.Pose().Translation();
    for (const auto &cluster : clusters)
    {
      const double distance = cluster.position.Distance(robotPosition).value();
      if (nearest < 0 || distance < nearest)
        nearest = distance;
    }
    frc::SmartDashboard::PutNumber("AutoDrive/SimNearestFuelDistance", nearest);
  }
  void AutoDrive::ResetSimulation()
  {
    for (auto &frame : simulationSensors_->frames)
      frame.reset();
    perception_->Clear();
  }
#endif
} // namespace season
