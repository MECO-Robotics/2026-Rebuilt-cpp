#include "season/drive/DriveBehaviorConfig.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "RobotContainer.h"
#include "season/detail/RobotRuntime.h"
#include "season/auto/RobotAutos.h"
#include "season/detail/RobotPlatform.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/drive/DriveConfig.h"
#include "season/RobotControlConfig.h"
#include "season/vision/PerceptionConfig.h"
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <numbers>
#include <algorithm>
#include <cmath>
#include <string_view>
using namespace units::literals;
namespace
{
  base::MotorConfig RackMotorConfig()
  {
    // Rack gearing is specified in motor rotations per metre. MotorIO expects
    // motor rotations per drum rotation; PositionJoint converts drum turns to metres.
    return season::hardware::MotorRates(
        {.ids = {season::hardware::kRackCanId},
         .reversed = {false},
         .gearRatio = season::hardware::kRackGearRatio * 2 * std::numbers::pi *
                      season::hardware::kRackDrumRadius,
         .currentLimit = season::hardware::kRackMotorCurrent,
         .canBus = ""},
        base::Vendor::SparkMax);
  }

  base::MotorConfig HoodMotorConfig()
  {
    return season::hardware::MotorRates(
        {.ids = {season::hardware::kHoodCanId},
         .reversed = {false},
         .gearRatio = season::hardware::kHoodGearRatio,
         .currentLimit = season::hardware::kHoodMotorCurrent,
         .canBus = ""},
        base::Vendor::SparkMax);
  }

  // Derive trench references from the official field tag layout.
  season::FieldGeometry Geometry(const frc::AprilTagFieldLayout &layout)
  {
    auto midpoint = [&](int a, int b)
    {
      auto p = layout.GetTagPose(a).value(), q = layout.GetTagPose(b).value();
      return frc::Translation2d{(p.X() + q.X()) / 2, (p.Y() + q.Y()) / 2};
    };
    return {layout.GetFieldLength().value(), layout.GetFieldWidth().value(), midpoint(22, 23), midpoint(6, 7)};
  }
} // namespace
// Fuse accepted observations into the drivetrain pose estimate.
void RobotContainer::AcceptVision(const base::VisionObservation &observation)
{
  drive_.AddVision(observation.pose, observation.timestamp, observation.stdDevs);
  lastAcceptedVision_ = frc::Timer::GetFPGATimestamp().value();
  // Limelight supplies a field anchor for the pinned VIO-only Quest receiver.
  if (vision_->Inputs(1).tracking && !vision_->Inputs(1).aligned)
    vision_->AlignToField(1, observation.pose);
}
// Pass non-owning subsystem references to season command factories.
season::Mechanisms RobotContainer::Mechanisms() { return {rack_, intake_, conveyor_, bottom_, top_, hood_, shooter_}; }
RobotContainer::~RobotContainer() = default;
RobotContainer::RobotContainer()
    : runtime_(std::make_unique<season::runtime::RobotRuntime>(*this)),
      drive_(season::hardware::DriveConfig()),
      rack_("IntakeRack",
            runtime_->Motors().Make(base::Vendor::SparkMax, season::hardware::MotorRates(RackMotorConfig(), base::Vendor::SparkMax)),
            season::hardware::JointGains(season::hardware::kRackGains),
            2 * std::numbers::pi * season::hardware::kRackDrumRadius),
      hood_("Hood", runtime_->Motors().Make(base::Vendor::SparkMax, season::hardware::MotorRates(HoodMotorConfig(), base::Vendor::SparkMax)),
            season::hardware::JointGains(season::hardware::kHoodGains),
            1, base::Gravity::Cosine),
      intake_("IntakeRoller", runtime_->Motors().Make(base::Vendor::SparkMax, season::hardware::MotorRates(season::hardware::kIntake.motor, base::Vendor::SparkMax)), season::hardware::kIntakeGains),
      conveyor_("Conveyor",
                runtime_->Motors().Make(base::Vendor::SparkMax, season::hardware::MotorRates(season::hardware::kConveyor.motor, base::Vendor::SparkMax)),
                season::hardware::kConveyorGains),
      bottom_("BottomIndexer",
                runtime_->Motors().Make(base::Vendor::TalonFX, season::hardware::MotorRates(season::hardware::kBottomIndexer.motor, base::Vendor::TalonFX)),
              season::hardware::kIndexerGains),
      top_("TopIndexer",
                runtime_->Motors().Make(base::Vendor::SparkMax, season::hardware::MotorRates(season::hardware::kTopIndexer.motor, base::Vendor::SparkMax)),
           season::hardware::kIndexerGains),
      shooter_("ShooterFlywheel",
                runtime_->Motors().Make(base::Vendor::TalonFX, season::hardware::MotorRates(season::hardware::kShooter.motor, base::Vendor::TalonFX)),
               season::hardware::kShooterGains),
      layout_(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)), field_(Geometry(layout_)),
      vision_(
          std::make_unique<base::VisionSystem>(runtime_->LocalizationSources(), [this](const auto &obs)
                                               { AcceptVision(obs); })),
      checks_(
          [this]
          {
            drive_.Stop();
            rack_.Stop();
            hood_.Stop();
            intake_.Stop();
            conveyor_.Stop();
            bottom_.Stop();
            top_.Stop();
            shooter_.Stop();
          },
          season::hardware::SystemCheckPath(),
          {.setupPrompt = "Confirm robot on blocks, no fuel, clear personnel and Disable operator ready"})
{
  // Keep mechanism feedback and control tied to the standard robot loop.
  rack_.UseExternalScheduling();
  hood_.UseExternalScheduling();
  intake_.UseExternalScheduling();
  conveyor_.UseExternalScheduling();
  bottom_.UseExternalScheduling();
  top_.UseExternalScheduling();
  shooter_.UseExternalScheduling();
  // Install the pathfinder before constructing commands that use it.
  season::autonomous::RobotAutos::ConfigureDrive(drive_);
  runtime_->Initialize();
  const bool ntInputSelected = season::drive::ntgamepadconfig::kEnabled;
  const bool useNtInput = runtime_->UseNtInput(ntInputSelected);
  if (useNtInput)
    ntGamepad_ = std::make_unique<base::NtGamepad>(nt::NetworkTableInstance::GetDefault(),
                                                  season::drive::ntgamepadconfig::kDefaultPeriodMs,
                                                  season::drive::ntgamepadconfig::kDefaultStaleMs);
  frc::SmartDashboard::PutString("Input/DriverSource", useNtInput ? "nt" : "driverstation");
  frc::SmartDashboard::PutBoolean("Input/NTGamepadEnabled", useNtInput);
  // Manual driving resumes whenever no other command owns the drivetrain.
  drive_.SetDefaultCommand(frc2::cmd::RunEnd([this]
                                             { DriveTeleop(false); }, [this]
                                             { drive_.Stop(); }, {&drive_}));
  // Perception uses the live intake extension and the shared match snapshot.
  autoDrive_ = std::make_unique<season::AutoDrive>(
      drive_, [this]
      { return rack_.Position(); },
      [this]() -> const base::season2026::MatchContext &
      { return matchContext_; },
      units::second_t{season::vision::config::kObjectDetectionPeriodSeconds},
      runtime_->AutoDriveInputs());
  runtime_->InitializeFeedback();
  // Bind controls before registering the named autonomous actions.
  ConfigureBindings();
  autonomous_ = std::make_unique<season::autonomous::RobotAutos>(*this);
  autonomous_->Configure();
  // Reserve all controlled mechanisms during a diagnostic run.
  checks_.ConfigureRequirements({&drive_, &rack_, &hood_, &shooter_, &intake_, &conveyor_, &bottom_, &top_});
  season::ConfigureSystemChecks(
      checks_, drive_, rack_, hood_, shooter_,
      {{"intake", &intake_}, {"conveyor", &conveyor_}, {"bottom-indexer", &bottom_}, {"top-indexer", &top_}},
      [this]
      { return vision_->Inputs().connected; });
  runtime_->FinishSetup();
}

// Maintain mechanism feedback and control at the standard robot-loop cadence.
void RobotContainer::ControlMechanisms()
{
  frc::SmartDashboard::PutString("Input/ActiveSource", activeDriverInput_);
  if (ntGamepad_)
  {
    const auto state = ntGamepad_->Read();
    frc::SmartDashboard::PutBoolean("Input/NTAccepted", state.accepted);
    frc::SmartDashboard::PutNumber("Input/NTAgeMs", std::isfinite(state.ageMs) ? state.ageMs : -1.0);
    frc::SmartDashboard::PutNumber("Input/NTSequence", static_cast<double>(state.sequence));
  }
  const auto kControlPeriod = season::hardware::kControlPeriod;
  rack_.Control(kControlPeriod);
  hood_.Control(kControlPeriod);
  intake_.Control(kControlPeriod);
  conveyor_.Control(kControlPeriod);
  bottom_.Control(kControlPeriod);
  top_.Control(kControlPeriod);
  shooter_.Control(kControlPeriod);
}

// Bind controls to commands with explicit ownership and cleanup.
void RobotContainer::ConfigureBindings()
{
  runtime_->BindReset();
  // LB selects target driving or offense according to the dashboard toggle.
  (controller_.LeftBumper() &&
   []
   {
     return frc::DriverStation::IsTeleopEnabled() &&
            !frc::SmartDashboard::GetBoolean("AutoDrive/OffenseEnabled", false);
   }).WhileTrue(autoDrive_->Command());
  (controller_.LeftBumper() &&
   []
   {
     return frc::DriverStation::IsTeleopEnabled() && frc::SmartDashboard::GetBoolean("AutoDrive/OffenseEnabled", false);
   }).WhileTrue(season::OffensiveAutoDrive(*autoDrive_, drive_, Mechanisms(), field_, fuelEstimate_));
  auto m = Mechanisms();
  // No idle default overwrites shooter setpoints between actions.
  (controller_.Start() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(frc2::cmd::RunOnce(
          [this]
          {
            // Driver forward is an operator frame, never a new field/vision pose.
            const bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
            driverHeadingOffset_ =
                drive_.Pose().Rotation() - frc::Rotation2d{units::radian_t{red ? std::numbers::pi : 0}};
          },
          {&drive_}));
  // Triggers acquire and eject while held.
  (controller_.RightTrigger() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Acquire(m));
  (controller_.LeftTrigger() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Eject(m).AlongWith(
          frc2::cmd::RunEnd([this]
                            { rack_.SetPosition(season::mechanismconfig::kRackDeploy); }, [this]
                            { rack_.Stop(); }, {&rack_})));
  // Feed while held; rumble reports shooter readiness.
  (controller_.RightBumper() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::AgitateAndFeed(
          m, [this](double intensity)
          { controller_.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble, intensity); }));
  // Either operator can hold the rack deploy and stow commands.
  (controller_.POVUp() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Deploy(m));
  (copilot_.POVUp() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Deploy(m));
  (controller_.POVDown() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Stow(m));
  (copilot_.POVDown() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(season::Stow(m));
  // Copilot left/right latches the rack position.
  (copilot_.POVLeft() &&
   []
   {
     return frc::DriverStation::IsTeleopEnabled();
   }).OnTrue(frc2::cmd::RunOnce([this]
                                { rack_.SetPosition(season::mechanismconfig::kRackDeploy); }, {&rack_}));
  (copilot_.POVRight() &&
   []
   {
     return frc::DriverStation::IsTeleopEnabled();
   }).OnTrue(frc2::cmd::RunOnce([this]
                                { rack_.SetPosition(season::mechanismconfig::kRackStow); }, {&rack_}));
  // A assists aiming while retaining driver translation.
  (controller_.A() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .WhileTrue(frc2::cmd::RunEnd([this]
                                   { DriveTeleop(true); },
                                   [this]
                                   {
                                     drive_.Stop();
                                     hood_.Stop();
                                     shooter_.Stop();
                                   },
                                   {&drive_, &hood_, &shooter_}));
  // X runs the hub preset; Y latches ferry setpoints; B stops the shooter.
  (controller_.X() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::DriverHubPreset(m));
  (copilot_.X() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::DriverHubPreset(m));
  (controller_.Y() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::shotconfig::kFerryHood, season::shotconfig::kFerryRps));
  (copilot_.Y() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::shotconfig::kFerryHood, season::shotconfig::kFerryRps));
  (copilot_.A() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::shotconfig::kTrenchHood, season::shotconfig::kTrenchRps));
  (controller_.B() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterIdle(m));
  (copilot_.B() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterIdle(m));
}
frc2::Command *RobotContainer::GetAutonomousCommand() { return autonomous_->Selected(); }
void RobotContainer::ScheduleAutonomous(frc2::Command *command) { runtime_->Schedule(command); }
void RobotContainer::CancelChecks() { checks_.Cancel("Robot mode changed"); }

void RobotContainer::SimulationPeriodic() { runtime_->Periodic(); }
void RobotContainer::PrepareAutonomous() { runtime_->Reset(); }
bool RobotContainer::AutonomousReady() const { return runtime_->ResetReady(); }
