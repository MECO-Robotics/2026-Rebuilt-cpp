#include "season/simulation/AutoDriveSimulation.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "season/simulation/SimulationConfig.h"
#include "season/drive/AutoDrive.h"
#include "season/vision/PerceptionConfig.h"
#include <base/FootprintAutoDrive.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace season::simulation {
namespace {
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
 }
AutoDriveSimulation::AutoDriveSimulation() {
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintProfileSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/AvoidBumpsSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/TrackTargetSim", false);
}
std::unique_ptr<base::ObjectDetectionIO> AutoDriveSimulation::Camera(
    std::size_t i, const std::string &, std::function<frc::Transform3d()> mount, units::second_t) {
  mounts_[i] = std::move(mount);
  meco::sim::PieceSensorConfig config;
  config.maxRange = autodriveconfig::kCameraRangeMetres;
  config.seed = runtimeconfig::kFuelRandomSeed + i;
  config.robotToCamera = mounts_[i]();
  sensors[i] = std::make_unique<meco::sim::SimulatedPieceSensor>(config);
  return std::make_unique<base::InjectedObjectDetectionIO>([this, i] { return std::exchange(frames[i], std::nullopt); });
}
bool AutoDriveSimulation::TrackTarget() const {
  return frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false);
}
std::optional<frc2::CommandPtr> AutoDriveSimulation::NavigationCommand(
    base::Drive &drive, std::function<std::optional<frc::Pose2d>()> target) {
  if (frc::SmartDashboard::GetBoolean("AutoDrive/UseFootprintSim", false))
  {
    base::FootprintAutoDriveConfig config;
    const bool avoidBumps = frc::SmartDashboard::GetBoolean("AutoDrive/AvoidBumpsSim", false);
    config.footprint = {autodriveconfig::kFootprintLengthMetres, autodriveconfig::kFootprintWidthMetres};
    config.field = {0, 0, simulation::kFieldLength, simulation::kFieldWidth};
    if (frc::SmartDashboard::GetBoolean("AutoDrive/FollowFuel", false) ||
        frc::SmartDashboard::GetBoolean("AutoDrive/TrackTargetSim", false))
      config.targetTranslationReplanThreshold = season::autodriveconfig::kTargetReplanDistanceMetres;
    // The modeled swerve can drift after a rotation if it releases the
    // drivetrain while residual angular speed is still appreciable.
    config.follow.headingTolerance = autodriveconfig::kHeadingToleranceRadians;
    config.follow.angularSettleSpeed = autodriveconfig::kAngularSettleSpeed;
    config.follow.settleSamples = autodriveconfig::kIntermediateSettleSamples;
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
    return base::AutoDriveFootprintToPose(drive, target, std::move(obstacles), config, outcome)
        .FinallyDo([entry, outcome](bool) mutable
                   { entry.SetString(FootprintOutcomeName(*outcome)); });
  }
  return std::nullopt;
}
void AutoDriveSimulation::Update(const frc::Pose3d &truth, const FuelModel &fuel, bool fieldColliders,
                                 const std::vector<base::ObjectCluster> &clusters, const base::Drive &drive) {
    std::vector<frc::Translation3d> points;
    points.reserve(fuel.FloorFuel().size());
    for (const auto &p : fuel.FloorFuel())
      points.emplace_back(units::meter_t{p.position.x}, units::meter_t{p.position.y}, units::meter_t{p.position.z});
    static const auto fieldObstacles = simulation::FieldColliders();
    static const std::vector<base::sim::BodySpec> noObstacles;
    const auto &obstacles = fieldColliders ? fieldObstacles : noObstacles;
    ++sequence;
    for (size_t i = 0; i < 2; ++i)
    {
      sensors[i]->SetMount(mounts_[i]());
      frames[i] = sensors[i]->ObserveFrame(
          truth, points, obstacles, frc::Timer::GetFPGATimestamp(), sequence);
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontDetections" : "AutoDrive/SimRearDetections",
                                     frames[i]->detections.size());
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontFrameAge" : "AutoDrive/SimRearFrameAge",
                                     (frc::Timer::GetFPGATimestamp() - frames[i]->captureTime).value());
    }
    frc::SmartDashboard::PutBoolean("AutoDrive/DetectionFresh", !clusters.empty());
    frc::SmartDashboard::PutNumber("AutoDrive/ClusterCount", clusters.size());
    double nearest = -1;
    const auto robotPosition = drive.Pose().Translation();
    for (const auto &cluster : clusters)
    {
      const double distance = cluster.position.Distance(robotPosition).value();
      if (nearest < 0 || distance < nearest)
        nearest = distance;
    }
    frc::SmartDashboard::PutNumber("AutoDrive/SimNearestFuelDistance", nearest);
}
void AutoDriveSimulation::Reset() { for (auto &frame : frames) frame.reset(); }
}
#endif
