#include "season/detail/RobotRuntimeInternal.h"
#ifndef __FRC_ROBORIO__
#include "RobotContainer.h"
#include "season/drive/DriveBehaviorConfig.h"
#include "season/detail/RobotPlatform.h"
#include "season/vision/VisionConfig.h"
#include "season/simulation/SimulationConfig.h"
#include "season/simulation/AutoDriveSimulation.h"
#include "season/simulation/Simulation.h"
#include "meco/sim/DriveScene.h"
#include "meco/sim/DriverInputTelemetry.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <stdexcept>

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
