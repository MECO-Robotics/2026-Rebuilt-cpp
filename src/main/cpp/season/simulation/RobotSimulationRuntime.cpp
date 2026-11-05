#include "season/detail/RobotRuntimeInternal.h"
#ifndef __FRC_ROBORIO__
#include "RobotContainer.h"
#include "season/drive/DriveBehaviorConfig.h"
#include "season/detail/RobotPlatform.h"
#include "season/vision/VisionConfig.h"
#include "season/simulation/SimulationConfig.h"
#include "season/drive/AutoDrive.h"
#include <meco/sim/SimulatedPieceCameraBank.h>
#include <base/FootprintAutoDrive.h>
#include <meco/sim/ColliderNavigation.h>
#include <meco/sim/FootprintNavigation.h>
#include <networktables/NetworkTableInstance.h>
#include <utility>
#include "season/simulation/Simulation.h"
#include "meco/sim/DriveScene.h"
#include "meco/sim/DriverInputTelemetry.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <stdexcept>

namespace season { class FuelModel; }
namespace season::simulation {
// Owns desktop-only camera models and optional simulation navigation policy.
class AutoDriveSimulation final : public AutoDriveEnvironment {
public:
  AutoDriveSimulation();
  std::unique_ptr<base::ObjectDetectionIO> Camera(std::size_t, const std::string &, std::function<frc::Transform3d()>, units::second_t) override;
  bool TrackTarget() const override;
  std::optional<frc2::CommandPtr> NavigationCommand(base::Drive &, std::function<std::optional<frc::Pose2d>()>) override;
  void Update(const frc::Pose3d &, const FuelModel &, bool, const std::vector<base::ObjectCluster> &, const base::Drive &);
  void Reset();
  void SetFieldCollidersEnabled(bool enabled) { fieldCollidersEnabled_ = enabled; }
private:
  meco::sim::SimulatedPieceCameraBank cameras_{2};
  bool fieldCollidersEnabled_ = true;
};
}

namespace season::simulation {
AutoDriveSimulation::AutoDriveSimulation() {
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/UseFootprintProfileSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/AvoidBumpsSim", false);
  frc::SmartDashboard::PutBoolean("AutoDrive/TrackTargetSim", false);
}
std::unique_ptr<base::ObjectDetectionIO> AutoDriveSimulation::Camera(
    std::size_t i, const std::string &, std::function<frc::Transform3d()> mount, units::second_t) {
  // Frames follow the simulation loop; the period controls the real NT subscription.
  meco::sim::PieceSensorConfig config;
  config.maxRange = autodriveconfig::kCameraRangeMetres;
  config.seed = runtimeconfig::kFuelRandomSeed + i;
  return cameras_.Configure(i, config, std::move(mount));
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
    const bool fieldColliders = fieldCollidersEnabled_;
    auto obstacles = [avoidBumps, fieldColliders]
    {
      std::vector<base::FieldRectangle> boxes;
      if (fieldColliders)
        for (const auto &rectangle : meco::sim::ColliderRectangles(
                 simulation::FieldColliders(), autodriveconfig::kCollisionBottomThresholdMetres,
                 autodriveconfig::kCollisionTopThresholdMetres))
          boxes.push_back({rectangle.minX, rectangle.minY, rectangle.maxX, rectangle.maxY});
      if (fieldColliders && avoidBumps)
      {
        const auto ramps = simulation::RampObstacles();
        boxes.insert(boxes.end(), ramps.begin(), ramps.end());
      }
      // Desktop HAL/NT tests can inject one perceived obstacle. It is
      // not a simulated rigid body or a hardware detection source.
      meco::sim::AppendInjectedFootprintObstacle(boxes, nt::NetworkTableInstance::GetDefault()
          .GetEntry("/Simulation/FootprintObstacle"));
      return boxes;
    };
    return meco::sim::FootprintCommandWithOutcome(drive, std::move(target), std::move(obstacles),
        config, nt::NetworkTableInstance::GetDefault().GetEntry("/AutoDrive/FootprintOutcome"),
        base::FootprintOutcomeName);
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
    const auto samples = cameras_.Update(truth, points, obstacles,
        [] { return frc::Timer::GetFPGATimestamp(); });
    for (size_t i = 0; i < samples.size(); ++i)
    {
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontDetections" : "AutoDrive/SimRearDetections",
                                     samples[i].detections);
      frc::SmartDashboard::PutNumber(i == 0 ? "AutoDrive/SimFrontFrameAge" : "AutoDrive/SimRearFrameAge",
                                     (frc::Timer::GetFPGATimestamp() - samples[i].captureTime).value());
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
void AutoDriveSimulation::Reset() { cameras_.Reset(); }
}

namespace season::runtime {
struct RobotRuntime::DesktopSimulation final : RobotRuntime::Simulation {
  explicit DesktopSimulation(RobotRuntime &runtime) : runtime_(runtime), owner_(runtime.owner_) {}

  VisionProviders LocalizationProviders() override {
    auto localization = [this] {
      if (!simulatedLocalization_) simulatedLocalization_ = std::make_shared<season::simulation::SimulatedLocalization>();
      return simulatedLocalization_;
    };
    return {[localization] { return localization()->limelight.MakeIO(); },
            [localization] { return localization()->quest.MakeIO(); }};
  }
  std::shared_ptr<season::AutoDriveEnvironment> AutoDriveInputs() override {
    autoDriveSimulation_ = std::make_shared<season::simulation::AutoDriveSimulation>();
    autoDriveSimulation_->SetFieldCollidersEnabled(simulationFieldCollidersEnabled_);
    return autoDriveSimulation_;
  }
  void Initialize() override {
    driveScene_ = meco::sim::MakeRuntimeDriveScene(owner_.drive_, [this] {
      return meco::sim::DriveScene::Configuration{
          .fieldCollidersEnabled = true,
          .feedbackLoops = season::simulation::runtimeconfig::kResetFeedbackLoops,
          .colliders = season::simulation::FieldColliders(),
          .contact = season::simulation::RobotContacts(),
          .clearStartingPose = [](const frc::Pose2d &pose) { return season::simulation::IsClearStartingPose(pose); },
          .resetSensors = [this] { if (simulatedLocalization_) simulatedLocalization_->Reset(); owner_.lastAcceptedVision_ = -1e9; }};
    });
    if (driveScene_) {
      driverInputTelemetry_ = std::make_unique<meco::sim::DriverInputTelemetry>(drive::ntgamepadconfig::kEnabled);
      frc::SmartDashboard::PutNumber("SimulationTuning/SteerKP", owner_.drive_.Config().steerKP);
      frc::SmartDashboard::PutNumber("SimulationTuning/SteerKD", owner_.drive_.Config().steerKD);
      frc::SmartDashboard::PutNumber("SimulationTuning/AutoTranslationKP", owner_.drive_.Config().autoTranslationKP);
      frc::SmartDashboard::PutNumber("SimulationTuning/AutoRotationKP", owner_.drive_.Config().autoRotationKP);
      simulationGroundedBodyHeight_ = units::meter_t{owner_.drive_.Config().simulation.rideHeight};
      simulationFieldCollidersEnabled_ = driveScene_->FieldCollidersEnabled();
      if (autoDriveSimulation_) autoDriveSimulation_->SetFieldCollidersEnabled(simulationFieldCollidersEnabled_);
    }
  }
  void InitializeFeedback() override {
    if (driveScene_) fuelSimulation_ = std::make_unique<season::FuelSimulation>(owner_.Mechanisms(), owner_.drive_, runtime_.state_->fuelMotorLoads_);
  }
  void FinishSetup() override { if (driveScene_) runtime_.Reset(); }
  bool UseNtInput(bool selected) const override { return driverInputTelemetry_ ? driverInputTelemetry_->Selected(selected) : selected; }
  void BindReset() override {
    if (driveScene_) owner_.controller_.Back().OnTrue(frc2::cmd::RunOnce([this] { runtime_.Reset(); }));
  }
  void PublishDriverInput(const base::NtGamepad::State &nt, double a, double b, double c, double x, double y, double omega) override {
    if (driverInputTelemetry_)
      driverInputTelemetry_->Publish(nt, a, b, c, x, y, omega, owner_.driverHeadingOffset_, static_cast<bool>(owner_.ntGamepad_));
  }
  const std::vector<int> *TagIds() const override { return simulatedLocalization_ ? &simulatedLocalization_->tagIds : nullptr; }
  bool Ready() const override { return !driveScene_ || driveScene_->Ready(); }
  void Schedule(frc2::Command &command) override {
    if (driveScene_) driveScene_->BeginAutonomousSchedule();
    command.Schedule();
    if (driveScene_ && driveScene_->EndAutonomousSchedule()) command.Cancel();
  }
  void Periodic() override {
    std::optional<season::FuelRobotState> fuelRobotState;
    if (auto snapshot = driveScene_ ? driveScene_->Update() : std::nullopt) {
      const auto &body = snapshot->body;
      const auto &visualPose = snapshot->visualPose;
      if (driveScene_->Ready() && simulatedLocalization_)
        simulatedLocalization_->Update(visualPose, frc::Timer::GetFPGATimestamp(), owner_.layout_,
            season::hardware::VisionConfig(owner_.layout_).robotToCamera, season::hardware::QuestConfig(owner_.layout_).robotToQuest);
      frc::SmartDashboard::PutBoolean("Localization/LimelightConnected", owner_.vision_->Inputs().connected);
      frc::SmartDashboard::PutBoolean("Localization/QuestTracking", owner_.vision_->Inputs(1).tracking);
      frc::SmartDashboard::PutBoolean("Localization/QuestAligned", owner_.vision_->Inputs(1).aligned);
      frc::SmartDashboard::PutNumber("Localization/LimelightTagCount", simulatedLocalization_ ? simulatedLocalization_->tagIds.size() : 0);
      frc::SmartDashboard::PutNumber("Localization/TruthErrorMetres", owner_.drive_.Pose().Translation().Distance(visualPose.ToPose2d().Translation()).value());
      const auto &config = owner_.drive_.Config().simulation;
      fuelRobotState = season::FuelRobotState{visualPose,
          {body.linearVelocity.x, body.linearVelocity.y, body.linearVelocity.z},
          {body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z},
          {config.centerOfMassX, config.centerOfMassY, config.centerOfMassZ + simulationGroundedBodyHeight_.value()}};
    }
    if (fuelSimulation_)
      fuelSimulation_->Periodic(season::simulation::runtimeconfig::kSimulationPeriodSeconds, fuelRobotState, owner_.matchContext_.ownActive);
    if (autoDriveSimulation_ && owner_.autoDrive_ && fuelSimulation_ && fuelRobotState && driveScene_->Ready())
      autoDriveSimulation_->Update(fuelRobotState->pose, fuelSimulation_->Model(), simulationFieldCollidersEnabled_, owner_.autoDrive_->VisibleFuel(), owner_.drive_);
  }
  void Reset() override {
    if (!driveScene_) return;
    const bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    const frc::Pose2d origin = simulationFieldCollidersEnabled_ ? season::simulation::DefaultSpawn(red) : frc::Pose2d{};
    if (simulationFieldCollidersEnabled_ && !season::simulation::IsClearStartingPose(origin))
      throw std::logic_error("Simulation spawn intersects field geometry");
    if (autoDriveSimulation_) autoDriveSimulation_->Reset();
    if (owner_.autoDrive_) owner_.autoDrive_->ClearPerception();
    if (simulatedLocalization_) simulatedLocalization_->Reset();
    owner_.lastAcceptedVision_ = -1e9;
    driveScene_->Reset(origin);
    if (fuelSimulation_) fuelSimulation_->Reset();
  }
private:
  RobotRuntime &runtime_;
  RobotContainer &owner_;
  std::shared_ptr<season::simulation::AutoDriveSimulation> autoDriveSimulation_;
  std::shared_ptr<season::simulation::SimulatedLocalization> simulatedLocalization_;
  std::unique_ptr<season::FuelSimulation> fuelSimulation_;
  std::unique_ptr<meco::sim::DriverInputTelemetry> driverInputTelemetry_;
  std::unique_ptr<meco::sim::DriveScene> driveScene_;
  units::meter_t simulationGroundedBodyHeight_{};
  bool simulationFieldCollidersEnabled_{true};
};
std::unique_ptr<RobotRuntime::Simulation> RobotRuntime::MakeDesktopSimulation() {
  return std::make_unique<DesktopSimulation>(*this);
}
} // namespace season::runtime
#else
std::unique_ptr<season::runtime::RobotRuntime::Simulation> season::runtime::RobotRuntime::MakeDesktopSimulation() { return {}; }
#endif
