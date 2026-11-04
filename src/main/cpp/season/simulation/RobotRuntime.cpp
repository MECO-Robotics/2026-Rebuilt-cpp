#include "RobotRuntime.h"
#include "RobotContainer.h"
#include "../mechanisms/RobotPlatform.h"
#include "season/vision/VisionConfig.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/simulation/SimulationConfig.h"
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <cmath>
#include <numbers>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#ifndef __FRC_ROBORIO__
#include "meco/sim/LoadedMotorIO.h"
#include "meco/sim/SimulationPose.h"
#include "season/simulation/MotorModelConfig.h"
#endif
std::unique_ptr<base::MotorIO> season::simulation::RobotRuntime::MakeMotor(const season::hardware::FlywheelConfig &c, base::Vendor vendor, MotorLoad loadedMotor) {
#ifndef __FRC_ROBORIO__
  if (season::hardware::Mode() == base::Mode::Sim && loadedMotor != MotorLoad::None) {
    std::size_t loadIndex{};
    switch (loadedMotor) {
      case MotorLoad::Conveyor: loadIndex = 0; break;
      case MotorLoad::Bottom: loadIndex = 1; break;
      case MotorLoad::Top: loadIndex = 2; break;
      case MotorLoad::Shooter: loadIndex = 3; break;
      case MotorLoad::None: return base::MakeMotorIO(season::hardware::Mode(), vendor, season::hardware::MotorRates(c.motor, vendor), c.inertiaKgMetersSquared);
    }
    constexpr double nominalVoltage = season::simulation::motorconfig::kNominalVoltage;
    const bool neo = vendor == base::Vendor::SparkMax;
    const double stallCurrent = neo ? season::simulation::motorconfig::kNeoStallCurrentAmps : season::simulation::motorconfig::kKrakenStallCurrentAmps;
    const double stallTorque = neo ? season::simulation::motorconfig::kNeoStallTorqueNm : season::simulation::motorconfig::kKrakenStallTorqueNm;
    const double freeSpeedRpm = neo ? season::simulation::motorconfig::kNeoFreeSpeedRpm : season::simulation::motorconfig::kKrakenFreeSpeedRpm;
    meco::sim::LoadedMotorModel model;
    model.resistanceOhms = nominalVoltage / stallCurrent;
    model.torqueConstantNmPerAmp = stallTorque / stallCurrent;
    model.velocityConstantRadPerSecondPerVolt = freeSpeedRpm * 2 * std::numbers::pi / 60 / nominalVoltage;
    model.outputInertiaKgM2 = c.inertiaKgMetersSquared;
    model.viscousDragNmPerRadPerSecond = season::simulation::motorconfig::kViscousDrag;
    model.maximumPhysicsStepSeconds = season::simulation::motorconfig::kMaxPhysicsStepSeconds;
    model.maximumUpdateSeconds = season::simulation::motorconfig::kMaxUpdateSeconds;
    model.loadDirectionSpeedThresholdRadPerSecond = season::simulation::motorconfig::kLoadDirectionSpeedThreshold;
    auto loads = fuelMotorLoads_;
    return base::MakeMotorIO(c.motor, model, [loads = std::move(loads), loadIndex] {
      return loads->torque[loadIndex];
    });
  }
#else
  (void)loadedMotor;
#endif
  return base::MakeMotorIO(season::hardware::Mode(), vendor, season::hardware::MotorRates(c.motor, vendor), c.inertiaKgMetersSquared);
}
// Select simulated observations or physical localization devices.
std::vector<std::unique_ptr<base::VisionIO>> season::simulation::RobotRuntime::LocalizationSources()
{
  std::vector<std::unique_ptr<base::VisionIO>> sources;
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation())
  {
    sources.push_back(std::make_unique<base::InjectedVisionIO>(
        [this](auto &inputs)
        {
          inputs = std::move(simulatedLocalization_.limelight);
          simulatedLocalization_.limelight.observations.clear();
        }));
    sources.push_back(std::make_unique<base::InjectedVisionIO>(
        [this](auto &inputs)
        {
          inputs = std::move(simulatedLocalization_.quest);
          simulatedLocalization_.quest.observations.clear();
        }));
    return sources;
  }
#endif
  sources.push_back(base::MakeLimelightVisionIO(season::hardware::VisionConfig(owner_.layout_), [this]
                                                { return owner_.drive_.Pose().Rotation(); }));
  sources.push_back(base::MakeQuestNavVisionIO(season::hardware::QuestConfig(owner_.layout_)));
  return sources;
}
void season::simulation::RobotRuntime::Initialize() {
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation())
  {
    auto ntInstance = nt::NetworkTableInstance::GetDefault();
    ntGamepadConsumedPublisher_ = ntInstance.GetDoubleArrayTopic("/Simulation/NTGamepad/Consumed").Publish();
    driverStationCommandPublisher_ = ntInstance.GetDoubleArrayTopic("/Simulation/NTGamepad/DriverStationCommand").Publish();
    const char *ntGamepadMode = std::getenv("FRC_SIM_NT_GAMEPAD");
    const bool simNtEnabled = ntGamepadMode ? std::string_view{ntGamepadMode} == "1"
                                            : drive::ntgamepadconfig::kEnabled;
    frc::SmartDashboard::PutBoolean("Simulation/NTGamepad/Enabled", simNtEnabled);
    frc::SmartDashboard::PutNumber("SimulationTuning/SteerKP", owner_.drive_.Config().steerKP);
    frc::SmartDashboard::PutNumber("SimulationTuning/SteerKD", owner_.drive_.Config().steerKD);
    frc::SmartDashboard::PutNumber("SimulationTuning/AutoTranslationKP", owner_.drive_.Config().autoTranslationKP);
    frc::SmartDashboard::PutNumber("SimulationTuning/AutoRotationKP", owner_.drive_.Config().autoRotationKP);
    simulationPosePublisher_ =
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/Simulation/RobotTruth").Publish();
    simulationBodyPosePublisher_ =
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/Simulation/PhysicsBodyTruth").Publish();
    // The drive owns one physics adapter and its optional field arena.
    const auto backend = base::sim::BackendFromEnvironment();
    const auto &physics = owner_.drive_.Config().simulation;
    // Hard-mounted wheels have a fixed geometric body-to-floor offset.
    simulationGroundedBodyHeight_ = units::meter_t{physics.rideHeight};
    frc::SmartDashboard::PutNumber("FieldSimulation/GroundedBodyHeight", physics.rideHeight);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperCornerRadius", backend == base::sim::Backend::Full3D
                                                                             ? season::simulation::RobotContacts().noodleDiameter / 2
                                                                             : base::sim::kBumperCornerRadius);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperBottomHeight", physics.rideHeight - physics.bumperHalfHeight);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperTopHeight", physics.rideHeight + physics.bumperHalfHeight);
    auto simulation = backend == base::sim::Backend::Full3D
                          ? meco::sim::CreateDriveSimulationAdapter(owner_.drive_.Config(), backend, season::simulation::RobotContacts())
                          : meco::sim::CreateDriveSimulationAdapter(owner_.drive_.Config(), backend);
    const char *fieldMode = std::getenv("FRC_SIM_FIELD_COLLIDERS");
    simulationFieldCollidersEnabled_ = !fieldMode || std::string_view{fieldMode} != "0";
    if (simulationFieldCollidersEnabled_)
    {
      auto &arena = simulation->CreateArena(base::sim::ArenaScene::Empty);
      season::simulation::InstallField(arena);
    }
    frc::SmartDashboard::PutBoolean("FieldSimulation/CollidersEnabled", simulationFieldCollidersEnabled_);
    frc::SmartDashboard::PutNumber("FieldSimulation/ColliderCount",
                                   simulationFieldCollidersEnabled_ ? season::simulation::FieldColliders().size() : 0);
    simulationSession_ = simulation.get(); // Drive owns the session for this container lifetime.
    owner_.drive_.SetSimulationAdapter(std::move(simulation));
    // A starting auto pose must move both simulation truth and odometry.
    owner_.drive_.SetPoseResetCallback(
        [this](const frc::Pose2d &pose)
        {
          if (captureAutonomousPoseReset_)
          {
            captureAutonomousPoseReset_ = false;
            if (!simulationFieldCollidersEnabled_ || season::simulation::IsClearStartingPose(pose))
            {
              simulationResetPhysicalOrigin_ = pose;
              simulatedLocalization_.Reset();
              owner_.lastAcceptedVision_ = -1e9;
              // Allow simulated encoders and gyro to catch up after teleporting.
              simulationResetFeedbackLoops_ = runtimeconfig::kResetFeedbackLoops;
              owner_.drive_.ResetSimulation(pose);
              frc::SmartDashboard::PutString("Simulation/AutoStartPose", "applied");
            }
            else
            {
              // Reject overlapping starts and restore the prior simulation origin.
              autonomousPoseBlocked_ = true;
              owner_.drive_.ResetPose(simulationResetPhysicalOrigin_);
              frc::SmartDashboard::PutString("Simulation/AutoStartPose", "blocked-by-field");
            }
            return;
          }
          if (simulationResetFeedbackLoops_ > 0)
            simulationResetPose_ = pose;
        });
  }
#endif
}
void season::simulation::RobotRuntime::InitializeFuel() {
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation()) fuelSimulation_ = std::make_unique<season::FuelSimulation>(owner_.Mechanisms(), owner_.drive_, fuelMotorLoads_);
#endif
}
void season::simulation::RobotRuntime::ResetIfSimulation() { if (IsSimulation()) Reset(); }
bool season::simulation::RobotRuntime::IsSimulation() const { return frc::RobotBase::IsSimulation(); }
bool season::simulation::RobotRuntime::UseNtInput(bool selected) const {
#ifndef __FRC_ROBORIO__
  const char *overrideMode = std::getenv("FRC_SIM_NT_GAMEPAD");
  if (frc::RobotBase::IsSimulation() && overrideMode) return std::string_view{overrideMode} == "1";
#endif
  return selected;
}
void season::simulation::RobotRuntime::BindReset() {
#ifndef __FRC_ROBORIO__
  owner_.controller_.Back().OnTrue(frc2::cmd::RunOnce([this] { Reset(); }));
#endif
}
void season::simulation::RobotRuntime::PublishDriverInput(const season::drive::NtGamepad::State &ntGamepadState, double dsLeftY, double dsLeftX, double dsRightX, double x, double y, double omega) {
#ifndef __FRC_ROBORIO__
  const int64_t consumeNtUs = nt::Now();
  const double fieldX = x * owner_.driverHeadingOffset_.Cos() - y * owner_.driverHeadingOffset_.Sin();
  const double fieldY = x * owner_.driverHeadingOffset_.Sin() + y * owner_.driverHeadingOffset_.Cos();
  if (ntGamepadConsumedPublisher_)
  {
    const std::array<double, 7> consumed{static_cast<double>(ntGamepadState.sequence),
                                         static_cast<double>(ntGamepadState.sourceTimestampUs),
                                         static_cast<double>(consumeNtUs), fieldX, fieldY, omega,
                                         ntGamepadState.accepted ? 1.0 : 0.0};
    ntGamepadConsumedPublisher_.Set(consumed, consumeNtUs);
  }
  if (driverStationCommandPublisher_)
  {
    const std::array<double, 7> driverStation{static_cast<double>(consumeNtUs), dsLeftY, dsLeftX, dsRightX,
                                              fieldX, fieldY, omega};
    driverStationCommandPublisher_.Set(driverStation, consumeNtUs);
  }
  frc::SmartDashboard::PutBoolean("Simulation/NTGamepad/Accepted",
                                  owner_.ntGamepad_ && ntGamepadState.accepted);
  frc::SmartDashboard::PutNumber("Simulation/NTGamepad/AgeMs",
                                 std::isfinite(ntGamepadState.ageMs) ? ntGamepadState.ageMs : -1.0);
  frc::SmartDashboard::PutNumber("Simulation/NTGamepad/Sequence", ntGamepadState.sequence);
#endif
#ifdef __FRC_ROBORIO__
 (void)ntGamepadState; (void)dsLeftY; (void)dsLeftX; (void)dsRightX; (void)x; (void)y; (void)omega;
#endif
}
const std::vector<int> *season::simulation::RobotRuntime::TagIds() const {
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation()) return &simulatedLocalization_.tagIds;
#endif
  return nullptr;
}
bool season::simulation::RobotRuntime::ResetReady() const {
#ifndef __FRC_ROBORIO__
 return simulationResetFeedbackLoops_ == 0;
#else
 return true;
#endif
}
// Capture only the initial pose reset made while the auto is scheduled.
void season::simulation::RobotRuntime::Schedule(frc2::Command *command)
{
#ifndef __FRC_ROBORIO__
  captureAutonomousPoseReset_ = frc::RobotBase::IsSimulation();
  autonomousPoseBlocked_ = false;
  if (captureAutonomousPoseReset_)
    frc::SmartDashboard::PutString("Simulation/AutoStartPose", "not-requested");
#endif
  command->Schedule();
#ifndef __FRC_ROBORIO__
  captureAutonomousPoseReset_ = false;
  if (autonomousPoseBlocked_)
    command->Cancel();
#endif
}
// Publish physical truth and feed simulated sensors once per loop.
void season::simulation::RobotRuntime::Periodic()
{
#ifndef __FRC_ROBORIO__
  std::optional<season::FuelRobotState> fuelRobotState;
  if (auto truth = owner_.drive_.SimulationTruth())
  {
    // Scene reset changes encoder and gyro raw inputs asynchronously. Reconcile
    // once after transport/odometry has consumed them. Preserve any autonomous
    // pose seed applied since reset and motion in that frame. No loop waits.
    if (simulationResetFeedbackLoops_ > 0 && --simulationResetFeedbackLoops_ == 0)
      owner_.drive_.ResetPose(simulationResetPose_.TransformBy(frc::Transform2d{
          simulationResetPhysicalOrigin_, frc::Pose2d{units::meter_t{truth->x}, units::meter_t{truth->y},
                                                      frc::Rotation2d{units::radian_t{truth->yaw}}}}));
    // Keep body-origin physics and floor-referenced visualization distinct.
    const auto body = simulationSession_->PhysicsTruth().body;
    const auto q = body.pose.rotation;
    const frc::Pose3d bodyPose{units::meter_t{body.pose.position.x}, units::meter_t{body.pose.position.y},
                               units::meter_t{body.pose.position.z},
                               frc::Rotation3d{frc::Quaternion{q.w, q.x, q.y, q.z}}};
    simulationBodyPosePublisher_.Set(bodyPose);
    const auto visualPose = meco::sim::FloorReferencedRobotPose(bodyPose, simulationGroundedBodyHeight_);
    simulationPosePublisher_.Set(visualPose);
    // Do not generate localization observations while reset feedback settles.
    if (simulationResetFeedbackLoops_ == 0)
      simulatedLocalization_.Update(visualPose, frc::Timer::GetFPGATimestamp(), owner_.layout_,
                                    season::hardware::VisionConfig(owner_.layout_).robotToCamera, season::hardware::QuestConfig(owner_.layout_).robotToQuest);
    frc::SmartDashboard::PutBoolean("Localization/LimelightConnected", owner_.vision_->Inputs().connected);
    frc::SmartDashboard::PutBoolean("Localization/QuestTracking", owner_.vision_->Inputs(1).tracking);
    frc::SmartDashboard::PutBoolean("Localization/QuestAligned", owner_.vision_->Inputs(1).aligned);
    frc::SmartDashboard::PutNumber("Localization/LimelightTagCount", simulatedLocalization_.tagIds.size());
    frc::SmartDashboard::PutNumber("Localization/TruthErrorMetres",
                                   owner_.drive_.Pose().Translation().Distance(visualPose.ToPose2d().Translation()).value());
    const auto &config = owner_.drive_.Config().simulation;
    // Launch and contact calculations consume physical pose and velocities.
    fuelRobotState = season::FuelRobotState{
        visualPose,
        {body.linearVelocity.x, body.linearVelocity.y, body.linearVelocity.z},
        {body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z},
        {config.centerOfMassX, config.centerOfMassY, config.centerOfMassZ + simulationGroundedBodyHeight_.value()}};
  }
  // Step fuel once, then generate camera observations from the updated scene.
  if (fuelSimulation_)
    fuelSimulation_->Periodic(runtimeconfig::kSimulationPeriodSeconds, fuelRobotState, owner_.matchContext_.ownActive);
  if (owner_.autoDrive_ && fuelSimulation_ && fuelRobotState && simulationResetFeedbackLoops_ == 0)
    owner_.autoDrive_->UpdateSimulation(fuelRobotState->pose, fuelSimulation_->Model(), simulationFieldCollidersEnabled_);
#endif
}
// Reset the scene and allow sensor feedback to settle before reconciliation.
void season::simulation::RobotRuntime::Reset()
{
  owner_.driverHeadingOffset_ = frc::Rotation2d{};
#ifndef __FRC_ROBORIO__
  // Reset to a clear alliance-specific starting pose.
  const bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
  simulationResetPhysicalOrigin_ =
      simulationFieldCollidersEnabled_ ? season::simulation::DefaultSpawn(red) : frc::Pose2d{};
  if (simulationFieldCollidersEnabled_ && !season::simulation::IsClearStartingPose(simulationResetPhysicalOrigin_))
    throw std::logic_error("Simulation spawn intersects field geometry");
  if (owner_.autoDrive_)
    owner_.autoDrive_->ResetSimulation();
  simulatedLocalization_.Reset();
  owner_.lastAcceptedVision_ = -1e9;
  simulationResetFeedbackLoops_ = runtimeconfig::kResetFeedbackLoops;
  owner_.drive_.ResetSimulation(simulationResetPhysicalOrigin_);
#endif

#ifndef __FRC_ROBORIO__
  if (fuelSimulation_)
    fuelSimulation_->Reset();
#endif
}
