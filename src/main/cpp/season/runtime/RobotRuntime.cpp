#include "season/runtime/RobotRuntimeInternal.h"
#include "RobotContainer.h"
#include "season/vision/VisionConfig.h"
#include "season/hardware/RobotPlatform.h"
#include <base/RuntimeTuning.h>
#include <base/RuntimeMode.h>
#include <utility>

season::runtime::RobotRuntime::RobotRuntime(RobotContainer &owner)
    : owner_(owner), state_(std::make_unique<State>()) {
  // The provider is not invoked in real or replay modes. Its constructor only
  // retains this runtime; robot subsystems are initialized later.
  state_->simulation_ = base::MakeRuntimeScene<Simulation>([this] { return MakeDesktopSimulation(); });
}
season::runtime::RobotRuntime::~RobotRuntime() = default;
base::MotorIOSelector &season::runtime::RobotRuntime::Motors() { return state_->motorSelector_; }
base::DriveConfig season::runtime::RobotRuntime::ConfigureDrive(base::DriveConfig config) {
  return base::ConfigureRuntimeDrive(std::move(config), [this](base::DriveConfig simulationConfig) {
    return state_->simulation_->ConfigureDrive(std::move(simulationConfig));
  });
}
std::shared_ptr<season::AutoDriveEnvironment> season::runtime::RobotRuntime::AutoDriveInputs() {
  return state_->simulation_->AutoDriveInputs();
}
std::vector<std::unique_ptr<base::VisionIO>> season::runtime::RobotRuntime::LocalizationSources() {
  auto [limelight, quest] = state_->simulation_->LocalizationProviders();
  std::vector<std::unique_ptr<base::VisionIO>> sources;
  sources.push_back(base::MakeRuntimeLimelightVisionIO(season::hardware::VisionConfig(owner_.layout_), [this]
      { return owner_.drive_.Pose().Rotation(); }, std::move(limelight)));
  sources.push_back(base::MakeRuntimeQuestNavVisionIO(season::hardware::QuestConfig(owner_.layout_), std::move(quest)));
  return sources;
}
void season::runtime::RobotRuntime::Initialize() { state_->simulation_->Initialize(); }
void season::runtime::RobotRuntime::InitializeFeedback() { state_->simulation_->InitializeFeedback(); }
void season::runtime::RobotRuntime::BeforeScheduler() { state_->simulation_->BeforeScheduler(); }
void season::runtime::RobotRuntime::AfterScheduler() { state_->simulation_->AfterScheduler(); }
void season::runtime::RobotRuntime::BeforeScene() { state_->simulation_->BeforeScene(); }
void season::runtime::RobotRuntime::AfterScene() { state_->simulation_->AfterScene(); }
void season::runtime::RobotRuntime::FinishSetup() { state_->simulation_->FinishSetup(); }
bool season::runtime::RobotRuntime::UseNtInput(bool selected) const { return state_->simulation_->UseNtInput(selected); }
std::optional<frc2::CommandPtr> season::runtime::RobotRuntime::PracticeMatchCommand() {
  return state_->simulation_->PracticeMatchCommand();
}
const std::function<void(const base::DriverInputState &, double, double, double, frc::Rotation2d)> *season::runtime::RobotRuntime::DriveRequestObserver() const {
  return state_->simulation_->DriveRequestObserver();
}
void season::runtime::RobotRuntime::BindReset() { state_->simulation_->BindReset(); }
const std::vector<int> *season::runtime::RobotRuntime::TagIds() const { return state_->simulation_->TagIds(); }
bool season::runtime::RobotRuntime::ResetReady() const { return state_->simulation_->Ready(); }
void season::runtime::RobotRuntime::Periodic() { state_->simulation_->Periodic(); }
void season::runtime::RobotRuntime::Reset() {
  owner_.driverHeadingOffset_ = frc::Rotation2d{};
  state_->simulation_->Reset();
}
