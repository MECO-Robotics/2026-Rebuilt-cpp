#include "season/detail/RobotRuntimeInternal.h"
#include "RobotContainer.h"
#include "season/vision/VisionConfig.h"
#include <utility>

season::runtime::RobotRuntime::RobotRuntime(RobotContainer &owner)
    : owner_(owner), state_(std::make_unique<State>()) {
  // The provider is not invoked in real or replay modes. Its constructor only
  // retains this runtime; robot subsystems are initialized later.
  state_->simulation_ = base::MakeRuntimeScene<Simulation>([this] { return MakeDesktopSimulation(); });
}
season::runtime::RobotRuntime::~RobotRuntime() = default;
base::MotorIOSelector &season::runtime::RobotRuntime::Motors() { return state_->motorSelector_; }
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
void season::runtime::RobotRuntime::FinishSetup() { state_->simulation_->FinishSetup(); }
bool season::runtime::RobotRuntime::UseNtInput(bool selected) const { return state_->simulation_->UseNtInput(selected); }
void season::runtime::RobotRuntime::BindReset() { state_->simulation_->BindReset(); }
void season::runtime::RobotRuntime::PublishDriverInput(const base::NtGamepad::State &nt, double a, double b, double c, double x, double y, double omega) {
  state_->simulation_->PublishDriverInput(nt, a, b, c, x, y, omega);
}
const std::vector<int> *season::runtime::RobotRuntime::TagIds() const { return state_->simulation_->TagIds(); }
bool season::runtime::RobotRuntime::ResetReady() const { return state_->simulation_->Ready(); }
void season::runtime::RobotRuntime::Schedule(frc2::Command *command) { state_->simulation_->Schedule(*command); }
void season::runtime::RobotRuntime::Periodic() { state_->simulation_->Periodic(); }
void season::runtime::RobotRuntime::Reset() {
  owner_.driverHeadingOffset_ = frc::Rotation2d{};
  state_->simulation_->Reset();
}
