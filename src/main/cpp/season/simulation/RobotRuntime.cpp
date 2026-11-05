#include "season/drive/DriveBehaviorConfig.h"
#include "season/detail/RobotRuntime.h"
#include "RobotContainer.h"
#include "season/detail/RobotPlatform.h"
#include "season/vision/VisionConfig.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/simulation/SimulationConfig.h"
#include "season/simulation/MotorSimulation.h"
#include <base/Replay.h>
#include <base/RuntimeMode.h>
#include <base/VisionIOSelector.h>
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#include <utility>
#ifndef __FRC_ROBORIO__
#include "season/simulation/AutoDriveSimulation.h"
#include "season/simulation/Simulation.h"
#include "meco/sim/DriveScene.h"
#include <networktables/StructTopic.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include "meco/sim/SimulationPose.h"
#endif
struct season::runtime::RobotRuntime::State
{
  std::shared_ptr<season::FuelMotorLoads> fuelMotorLoads_ = std::make_shared<season::FuelMotorLoads>();
  base::MotorIOSelector motorSelector_{season::simulation::MotorProvider(fuelMotorLoads_)};
#ifndef __FRC_ROBORIO__
  std::shared_ptr<season::simulation::AutoDriveSimulation> autoDriveSimulation_;
  std::shared_ptr<season::simulation::SimulatedLocalization> simulatedLocalization_;
  std::unique_ptr<season::FuelSimulation> fuelSimulation_;
  nt::DoubleArrayPublisher ntGamepadConsumedPublisher_;
  nt::DoubleArrayPublisher driverStationCommandPublisher_;
  std::unique_ptr<meco::sim::DriveScene> driveScene_;
  units::meter_t simulationGroundedBodyHeight_{};
  bool simulationFieldCollidersEnabled_{true};
#endif
};
season::runtime::RobotRuntime::RobotRuntime(RobotContainer &owner)
    : owner_(owner), state_(std::make_unique<State>()) {}
season::runtime::RobotRuntime::~RobotRuntime() = default;
base::MotorIOSelector &season::runtime::RobotRuntime::Motors() { return state_->motorSelector_; }
std::shared_ptr<season::AutoDriveEnvironment> season::runtime::RobotRuntime::AutoDriveInputs() {
  return base::SelectRuntimeIO<std::shared_ptr<season::AutoDriveEnvironment>>(
      [] { return std::shared_ptr<season::AutoDriveEnvironment>{}; },
      [this]() -> std::shared_ptr<season::AutoDriveEnvironment> {
#ifndef __FRC_ROBORIO__
        state_->autoDriveSimulation_ = std::make_shared<season::simulation::AutoDriveSimulation>();
        return state_->autoDriveSimulation_;
#else
        return {};
#endif
      }, [] { return std::shared_ptr<season::AutoDriveEnvironment>{}; });
}
// Select simulated observations or physical localization devices.
std::vector<std::unique_ptr<base::VisionIO>> season::runtime::RobotRuntime::LocalizationSources()
{
  std::vector<std::unique_ptr<base::VisionIO>> sources;
  base::SimVisionProvider limelightSimulation, questSimulation;
#ifndef __FRC_ROBORIO__
  auto localization = [this] {
    if (!state_->simulatedLocalization_)
      state_->simulatedLocalization_ = std::make_shared<season::simulation::SimulatedLocalization>();
    return state_->simulatedLocalization_;
  };
  limelightSimulation = [localization] { return localization()->limelight.MakeIO(); };
  questSimulation = [localization] { return localization()->quest.MakeIO(); };
#endif
  sources.push_back(base::MakeRuntimeLimelightVisionIO(season::hardware::VisionConfig(owner_.layout_), [this]
                                                { return owner_.drive_.Pose().Rotation(); }, std::move(limelightSimulation)));
  sources.push_back(base::MakeRuntimeQuestNavVisionIO(season::hardware::QuestConfig(owner_.layout_), std::move(questSimulation)));
  return sources;
}
void season::runtime::RobotRuntime::Initialize() {
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation() && !base::replay::Active())
  {
    auto ntInstance = nt::NetworkTableInstance::GetDefault();
    state_->ntGamepadConsumedPublisher_ = ntInstance.GetDoubleArrayTopic("/Simulation/NTGamepad/Consumed").Publish();
    state_->driverStationCommandPublisher_ = ntInstance.GetDoubleArrayTopic("/Simulation/NTGamepad/DriverStationCommand").Publish();
    const char *ntGamepadMode = std::getenv("FRC_SIM_NT_GAMEPAD");
    const bool simNtEnabled = ntGamepadMode ? std::string_view{ntGamepadMode} == "1"
                                            : drive::ntgamepadconfig::kEnabled;
    frc::SmartDashboard::PutBoolean("Simulation/NTGamepad/Enabled", simNtEnabled);
    frc::SmartDashboard::PutNumber("SimulationTuning/SteerKP", owner_.drive_.Config().steerKP);
    frc::SmartDashboard::PutNumber("SimulationTuning/SteerKD", owner_.drive_.Config().steerKD);
    frc::SmartDashboard::PutNumber("SimulationTuning/AutoTranslationKP", owner_.drive_.Config().autoTranslationKP);
    frc::SmartDashboard::PutNumber("SimulationTuning/AutoRotationKP", owner_.drive_.Config().autoRotationKP);
    const auto &physics = owner_.drive_.Config().simulation;
    state_->simulationGroundedBodyHeight_ = units::meter_t{physics.rideHeight};
    state_->driveScene_ = std::make_unique<meco::sim::DriveScene>(owner_.drive_,
        meco::sim::DriveScene::Configuration{
            .fieldCollidersEnabled = true,
            .feedbackLoops = season::simulation::runtimeconfig::kResetFeedbackLoops,
            .colliders = season::simulation::FieldColliders(),
            .contact = season::simulation::RobotContacts(),
            .clearStartingPose = [](const frc::Pose2d &pose) { return season::simulation::IsClearStartingPose(pose); },
            .resetSensors = [this] { if (state_->simulatedLocalization_) state_->simulatedLocalization_->Reset(); owner_.lastAcceptedVision_ = -1e9; }});
    state_->simulationFieldCollidersEnabled_ = state_->driveScene_->FieldCollidersEnabled();
  }
#endif
}
void season::runtime::RobotRuntime::InitializeFeedback() {
#ifndef __FRC_ROBORIO__
  if (frc::RobotBase::IsSimulation() && !base::replay::Active()) state_->fuelSimulation_ = std::make_unique<season::FuelSimulation>(owner_.Mechanisms(), owner_.drive_, state_->fuelMotorLoads_);
#endif
}
void season::runtime::RobotRuntime::FinishSetup() { if (IsSimulation() && !base::replay::Active()) Reset(); }
bool season::runtime::RobotRuntime::IsSimulation() const { return frc::RobotBase::IsSimulation(); }
bool season::runtime::RobotRuntime::UseNtInput(bool selected) const {
#ifndef __FRC_ROBORIO__
  const char *overrideMode = std::getenv("FRC_SIM_NT_GAMEPAD");
  if (frc::RobotBase::IsSimulation() && overrideMode) return std::string_view{overrideMode} == "1";
#endif
  return selected;
}
void season::runtime::RobotRuntime::BindReset() {
#ifndef __FRC_ROBORIO__
  owner_.controller_.Back().OnTrue(frc2::cmd::RunOnce([this] { Reset(); }));
#endif
}
void season::runtime::RobotRuntime::PublishDriverInput(const base::NtGamepad::State &ntGamepadState, double dsLeftY, double dsLeftX, double dsRightX, double x, double y, double omega) {
#ifndef __FRC_ROBORIO__
  const int64_t consumeNtUs = nt::Now();
  const double fieldX = x * owner_.driverHeadingOffset_.Cos() - y * owner_.driverHeadingOffset_.Sin();
  const double fieldY = x * owner_.driverHeadingOffset_.Sin() + y * owner_.driverHeadingOffset_.Cos();
  if (state_->ntGamepadConsumedPublisher_)
  {
    const std::array<double, 7> consumed{static_cast<double>(ntGamepadState.sequence),
                                         static_cast<double>(ntGamepadState.sourceTimestampUs),
                                         static_cast<double>(consumeNtUs), fieldX, fieldY, omega,
                                         ntGamepadState.accepted ? 1.0 : 0.0};
    state_->ntGamepadConsumedPublisher_.Set(consumed, consumeNtUs);
  }
  if (state_->driverStationCommandPublisher_)
  {
    const std::array<double, 7> driverStation{static_cast<double>(consumeNtUs), dsLeftY, dsLeftX, dsRightX,
                                              fieldX, fieldY, omega};
    state_->driverStationCommandPublisher_.Set(driverStation, consumeNtUs);
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
const std::vector<int> *season::runtime::RobotRuntime::TagIds() const {
#ifndef __FRC_ROBORIO__
  if (state_->simulatedLocalization_) return &state_->simulatedLocalization_->tagIds;
#endif
  return nullptr;
}
bool season::runtime::RobotRuntime::ResetReady() const {
#ifndef __FRC_ROBORIO__
 return !state_->driveScene_ || state_->driveScene_->Ready();
#else
 return true;
#endif
}
// Capture only the initial pose reset made while the auto is scheduled.
void season::runtime::RobotRuntime::Schedule(frc2::Command *command)
{
#ifndef __FRC_ROBORIO__
  if (state_->driveScene_) state_->driveScene_->BeginAutonomousSchedule();
#endif
  command->Schedule();
#ifndef __FRC_ROBORIO__
  if (state_->driveScene_ && state_->driveScene_->EndAutonomousSchedule())
    command->Cancel();
#endif
}
// Publish physical truth and feed simulated sensors once per loop.
void season::runtime::RobotRuntime::Periodic()
{
#ifndef __FRC_ROBORIO__
  std::optional<season::FuelRobotState> fuelRobotState;
  if (auto snapshot = state_->driveScene_ ? state_->driveScene_->Update() : std::nullopt)
  {
    const auto &body = snapshot->body;
    const auto &visualPose = snapshot->visualPose;
    // Do not generate localization observations while reset feedback settles.
    if (state_->driveScene_->Ready() && state_->simulatedLocalization_)
      state_->simulatedLocalization_->Update(visualPose, frc::Timer::GetFPGATimestamp(), owner_.layout_,
                                    season::hardware::VisionConfig(owner_.layout_).robotToCamera, season::hardware::QuestConfig(owner_.layout_).robotToQuest);
    frc::SmartDashboard::PutBoolean("Localization/LimelightConnected", owner_.vision_->Inputs().connected);
    frc::SmartDashboard::PutBoolean("Localization/QuestTracking", owner_.vision_->Inputs(1).tracking);
    frc::SmartDashboard::PutBoolean("Localization/QuestAligned", owner_.vision_->Inputs(1).aligned);
    frc::SmartDashboard::PutNumber("Localization/LimelightTagCount", state_->simulatedLocalization_ ? state_->simulatedLocalization_->tagIds.size() : 0);
    frc::SmartDashboard::PutNumber("Localization/TruthErrorMetres",
                                   owner_.drive_.Pose().Translation().Distance(visualPose.ToPose2d().Translation()).value());
    const auto &config = owner_.drive_.Config().simulation;
    // Launch and contact calculations consume physical pose and velocities.
    fuelRobotState = season::FuelRobotState{
        visualPose,
        {body.linearVelocity.x, body.linearVelocity.y, body.linearVelocity.z},
        {body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z},
        {config.centerOfMassX, config.centerOfMassY, config.centerOfMassZ + state_->simulationGroundedBodyHeight_.value()}};
  }
  // Step fuel once, then generate camera observations from the updated scene.
  if (state_->fuelSimulation_)
    state_->fuelSimulation_->Periodic(season::simulation::runtimeconfig::kSimulationPeriodSeconds, fuelRobotState, owner_.matchContext_.ownActive);
  if (state_->autoDriveSimulation_ && owner_.autoDrive_ && state_->fuelSimulation_ && fuelRobotState && state_->driveScene_->Ready())
    state_->autoDriveSimulation_->Update(fuelRobotState->pose, state_->fuelSimulation_->Model(), state_->simulationFieldCollidersEnabled_,
                                 owner_.autoDrive_->VisibleFuel(), owner_.drive_);
#endif
}
// Reset the scene and allow sensor feedback to settle before reconciliation.
void season::runtime::RobotRuntime::Reset()
{
  owner_.driverHeadingOffset_ = frc::Rotation2d{};
#ifndef __FRC_ROBORIO__
  // Reset to a clear alliance-specific starting pose.
  const bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
  const frc::Pose2d origin =
      state_->simulationFieldCollidersEnabled_ ? season::simulation::DefaultSpawn(red) : frc::Pose2d{};
  if (state_->simulationFieldCollidersEnabled_ && !season::simulation::IsClearStartingPose(origin))
    throw std::logic_error("Simulation spawn intersects field geometry");
  if (state_->autoDriveSimulation_)
    state_->autoDriveSimulation_->Reset();
  if (owner_.autoDrive_)
    owner_.autoDrive_->ClearPerception();
  if (state_->simulatedLocalization_) state_->simulatedLocalization_->Reset();
  owner_.lastAcceptedVision_ = -1e9;
  if (state_->driveScene_) state_->driveScene_->Reset(origin);
#endif

#ifndef __FRC_ROBORIO__
  if (state_->fuelSimulation_)
    state_->fuelSimulation_->Reset();
#endif
}
