#include "season/simulation/AutoDriveSimulation.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "season/simulation/SimulationConfig.h"
#include "season/drive/AutoDrive.h"
#include "season/vision/PerceptionConfig.h"
#include <base/FootprintAutoDrive.h>
#include <meco/sim/ColliderNavigation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace season::simulation {
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
  sensors[i] = std::make_unique<meco::sim::SimulatedPieceSensorBridge>(config);
  return sensors[i]->MakeIO();
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
        for (const auto &rectangle : meco::sim::ColliderRectangles(
                 simulation::FieldColliders(), autodriveconfig::kCollisionBottomThresholdMetres,
                 autodriveconfig::kCollisionTopThresholdMetres))
          boxes.push_back({rectangle.minX, rectangle.minY, rectangle.maxX, rectangle.maxY});
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
                   { entry.SetString(base::FootprintOutcomeName(*outcome)); });
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
      const auto &frame = sensors[i]->Update(
          truth, mounts_[i](), points, obstacles, frc::Timer::GetFPGATimestamp(), sequence);
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontDetections" : "AutoDrive/SimRearDetections",
                                     frame.detections.size());
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontFrameAge" : "AutoDrive/SimRearFrameAge",
                                     (frc::Timer::GetFPGATimestamp() - frame.captureTime).value());
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
void AutoDriveSimulation::Reset() { for (auto &sensor : sensors) if (sensor) sensor->Reset(); }
}
#endif
