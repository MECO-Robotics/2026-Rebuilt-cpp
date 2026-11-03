#include "season/drive/OffensiveAutoDrive.h"
#include "base/SimulationLatency.h"
#include "RobotContainer.h"
#include "season/simulation/RobotRuntime.h"
#include "season/auto/RobotAutos.h"
#include "season/mechanisms/RobotPlatform.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/RobotConfiguration.h"
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
#include <cstdlib>
#include <string_view>
#include <stdexcept>
using namespace units::literals;
namespace
{
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
    : simulationRuntime_(std::make_unique<season::simulation::RobotRuntime>(*this)),
      drive_(season::hardware::DriveConfig(season::hardware::FastControlEnabled())),
      rack_("IntakeRack",
            base::MakeMotorIO(
                season::hardware::Mode(), base::Vendor::SparkMax,
                season::hardware::MotorRates({{season::hardware::kRackCanId},
                            {false},
                            season::hardware::kRackGearRatio * 2 * std::numbers::pi *
                                season::hardware::kRackDrumRadius,
                            season::hardware::kRackMotorCurrent,
                            ""},
                           base::Vendor::SparkMax),
                season::hardware::kRackMotorInertia),
            season::hardware::JointGains(season::hardware::kRackGains),
            2 * std::numbers::pi * season::hardware::kRackDrumRadius),
      hood_("Hood", base::MakeMotorIO(season::hardware::Mode(), base::Vendor::SparkMax, season::hardware::MotorRates({{season::hardware::kHoodCanId}, {false}, season::hardware::kHoodGearRatio, season::hardware::kHoodMotorCurrent, ""}, base::Vendor::SparkMax), season::hardware::kHoodMotorInertia),
            season::hardware::JointGains(season::hardware::kHoodGains),
            1, base::Gravity::Cosine),
      intake_("IntakeRoller", simulationRuntime_->MakeMotor(season::hardware::kIntake, base::Vendor::SparkMax, season::simulation::MotorLoad::None), season::hardware::kIntakeGains),
      conveyor_("Conveyor",
                simulationRuntime_->MakeMotor(season::hardware::kConveyor, base::Vendor::SparkMax, season::simulation::MotorLoad::Conveyor),
                season::hardware::kConveyorGains),
      bottom_("BottomIndexer",
                simulationRuntime_->MakeMotor(season::hardware::kBottomIndexer, base::Vendor::TalonFX, season::simulation::MotorLoad::Bottom),
              season::hardware::kIndexerGains),
      top_("TopIndexer",
                simulationRuntime_->MakeMotor(season::hardware::kTopIndexer, base::Vendor::SparkMax, season::simulation::MotorLoad::Top),
           season::hardware::kIndexerGains),
      shooter_("ShooterFlywheel",
                simulationRuntime_->MakeMotor(season::hardware::kShooter, base::Vendor::TalonFX, season::simulation::MotorLoad::Shooter),
               season::hardware::kShooterGains),
      layout_(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)), field_(Geometry(layout_)),
      vision_(
          std::make_unique<base::VisionSystem>(simulationRuntime_->LocalizationSources(), [this](const auto &obs)
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
  fastControlEnabled_ = season::hardware::FastControlEnabled();
  fastMechanismCanFd_ = season::hardware::MechanismCanFd();
  // The command scheduler may run faster than the mechanisms' validated
  // 20 ms controller interval. Keep their feedback/control cadence explicit.
  rack_.UseExternalScheduling();
  hood_.UseExternalScheduling();
  intake_.UseExternalScheduling();
  conveyor_.UseExternalScheduling();
  bottom_.UseExternalScheduling();
  top_.UseExternalScheduling();
  shooter_.UseExternalScheduling();
  // Install the pathfinder before constructing commands that use it.
  season::autonomous::RobotAutos::ConfigureDrive(drive_);
  simulationRuntime_->Initialize();
  const bool ntInputSelected = fastControlProfile_.driverInput == "nt" && fastControlProfile_.ntGamepadEnabled;
  const bool useNtInput = simulationRuntime_->UseNtInput(ntInputSelected);
  if (useNtInput)
    ntGamepad_ = std::make_unique<season::drive::NtGamepad>(nt::NetworkTableInstance::GetDefault(),
                                                            fastControlProfile_.ntPeriodMs,
                                                            fastControlProfile_.ntStaleMs);
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
      units::second_t{season::drive::fastcontrol::Config().detectionPeriodMs / 1000.0});
  simulationRuntime_->InitializeFuel();
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
  simulationRuntime_->ResetIfSimulation();
}

// Maintain mechanism feedback and control at 50 Hz independently of the
// drivetrain/command-scheduler cadence. Call from the 20 ms TimedRobot loop.
void RobotContainer::ControlMechanisms20ms()
{
  frc::SmartDashboard::PutString("Input/ActiveSource", activeDriverInput_);
  if (ntGamepad_)
  {
    const auto state = ntGamepad_->Read();
    frc::SmartDashboard::PutBoolean("Input/NTAccepted", state.accepted);
    frc::SmartDashboard::PutNumber("Input/NTAgeMs", std::isfinite(state.ageMs) ? state.ageMs : -1.0);
    frc::SmartDashboard::PutNumber("Input/NTSequence", static_cast<double>(state.sequence));
  }
  if (fastControlEnabled_)
    return;
  ++sparkControlCount_;
  ++talonControlCount_;
  const auto kControlPeriod = season::hardware::kControlPeriod;
  rack_.Control(kControlPeriod);
  hood_.Control(kControlPeriod);
  intake_.Control(kControlPeriod);
  conveyor_.Control(kControlPeriod);
  bottom_.Control(kControlPeriod);
  top_.Control(kControlPeriod);
  shooter_.Control(kControlPeriod);
}

// Tick the selected groups at exact integer submultiples of the 2 ms scheduler.
void RobotContainer::ControlMechanismsFast()
{
  if (!fastControlEnabled_)
    return;
  ++fastMechanismControlTick_;
  const auto &profile = fastControlProfile_;
  const uint64_t sparkDivider = static_cast<uint64_t>(std::llround(profile.sparkControlMs / profile.periodMs));
  if (fastMechanismControlTick_ % sparkDivider == 0)
  {
    ++sparkControlCount_;
    const units::second_t kSparkControlPeriod{profile.sparkControlMs / 1000.0};
    rack_.Control(kSparkControlPeriod);
    hood_.Control(kSparkControlPeriod);
    intake_.Control(kSparkControlPeriod);
    conveyor_.Control(kSparkControlPeriod);
    top_.Control(kSparkControlPeriod);
  }
  const double talonPeriodMs = fastMechanismCanFd_ ? profile.talonFdControlMs : profile.talonClassicControlMs;
  const uint64_t talonDivider = static_cast<uint64_t>(std::llround(talonPeriodMs / profile.periodMs));
  if (fastMechanismControlTick_ % talonDivider == 0)
  {
    ++talonControlCount_;
    const units::second_t talonControlPeriod{talonPeriodMs / 1000.0};
    bottom_.Control(talonControlPeriod);
    shooter_.Control(talonControlPeriod);
  }
}

// Bind controls to commands with explicit ownership and cleanup.
void RobotContainer::ConfigureBindings()
{
  simulationRuntime_->BindReset();
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
  // Presets latch between button presses. No idle default overwrites their setpoints.
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
                            { rack_.SetPosition(season::hardware::kRackDeploy); }, [this]
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
                                { rack_.SetPosition(season::hardware::kRackDeploy); }, {&rack_}));
  (copilot_.POVRight() &&
   []
   {
     return frc::DriverStation::IsTeleopEnabled();
   }).OnTrue(frc2::cmd::RunOnce([this]
                                { rack_.SetPosition(season::hardware::kRackStow); }, {&rack_}));
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
  // Face buttons select latched shooter presets or stop the shooter.
  (controller_.X() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::DriverHubPreset(m));
  (copilot_.X() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::DriverHubPreset(m));
  (controller_.Y() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::hardware::kFerryHood, season::hardware::kFerryRps));
  (copilot_.Y() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::hardware::kFerryHood, season::hardware::kFerryRps));
  (copilot_.A() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterPreset(m, season::hardware::kTrenchHood, season::hardware::kTrenchRps));
  (controller_.B() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterIdle(m));
  (copilot_.B() && []
   { return frc::DriverStation::IsTeleopEnabled(); })
      .OnTrue(season::ShooterIdle(m));
}
// Shape driver inputs before applying the driver-relative field frame.
void RobotContainer::DriveTeleop(bool aim)
{
  // Preserve MecoBase 2026.2.1's per-axis shaping before rotating the driver
  // translation frame. FieldDrive still owns alliance perspective and gating.
  const auto deadband = [](double value)
  { return std::abs(value) < season::hardware::kDriverDeadband ? 0. : std::copysign((std::abs(value) - season::hardware::kDriverDeadband) / (1 - season::hardware::kDriverDeadband), value); };
  const bool dsConnected = controller_.GetHID().IsConnected() && controller_.GetHID().GetAxisCount() >= 5;
  const double dsLeftY = dsConnected ? controller_.GetLeftY() : 0.0;
  const double dsLeftX = dsConnected ? controller_.GetLeftX() : 0.0;
  const double dsRightX = dsConnected ? controller_.GetRightX() : 0.0;
  const auto validAxis = [](double value)
  { return std::isfinite(value) && std::abs(value) <= 1.0; };
  const bool dsInputValid = dsConnected &&
                            validAxis(dsLeftY) && validAxis(dsLeftX) && validAxis(dsRightX);
  double rawLeftY = 0.0, rawLeftX = 0.0, rawRightX = 0.0;
  bool inputAvailable = false;
  activeDriverInput_ = "none";
  season::drive::NtGamepad::State ntGamepadState;
  if (ntGamepad_)
    ntGamepadState = ntGamepad_->Read();
  if (ntGamepad_ && ntGamepadState.accepted)
  {
    rawLeftY = ntGamepadState.leftY;
    rawLeftX = ntGamepadState.leftX;
    rawRightX = ntGamepadState.rightX;
    inputAvailable = true;
    activeDriverInput_ = "nt";
  }
  else if (dsInputValid && (!ntGamepad_ || fastControlProfile_.ntFallbackToDriverStation))
  {
    rawLeftY = dsLeftY;
    rawLeftX = dsLeftX;
    rawRightX = dsRightX;
    inputAvailable = true;
    activeDriverInput_ = "driverstation";
  }
  const bool teleopAuthority = frc::DriverStation::IsTeleopEnabled() && !frc::DriverStation::IsEStopped();
  double x = deadband(base::simulation::ObserveJoystick(-rawLeftY)) * drive_.Config().maxSpeed;
  double y = deadband(-rawLeftX) * drive_.Config().maxSpeed;
  double omega = deadband(-rawRightX) * drive_.Config().maxAngularRate;
  if (!teleopAuthority || !inputAvailable)
    x = y = omega = 0.0;
  // Aim controls rotation and shooter targets while translation stays manual.
  if (aim && teleopAuthority && inputAvailable)
  {
    const auto p = drive_.Pose();
    const auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
    const auto shot = season::CalculateDriverAim(alliance, p, field_, simulationRuntime_->IsSimulation());
    hood_.SetPosition(shot.hoodRotations);
    shooter_.SetVelocity(shot.flywheelRps);
    const double error = std::remainder(shot.headingRadians - p.Rotation().Radians().value(), 2 * std::numbers::pi);
    omega = std::clamp(season::hardware::kAimHeadingGain * error, -drive_.Config().maxAngularRate, drive_.Config().maxAngularRate);
  }
  drive_.FieldDrive(x * driverHeadingOffset_.Cos() - y * driverHeadingOffset_.Sin(),
                    x * driverHeadingOffset_.Sin() + y * driverHeadingOffset_.Cos(), omega);
  simulationRuntime_->PublishDriverInput(ntGamepadState, dsLeftY, dsLeftX, dsRightX, x, y, omega);
}
frc2::Command *RobotContainer::GetAutonomousCommand() { return autonomous_->Selected(); }
void RobotContainer::ScheduleAutonomous(frc2::Command *command) { simulationRuntime_->Schedule(command); }
void RobotContainer::CancelChecks() { checks_.Cancel("Robot mode changed"); }

void RobotContainer::SimulationPeriodic() { simulationRuntime_->Periodic(); }
void RobotContainer::ResetSimulation() { simulationRuntime_->Reset(); }
bool RobotContainer::SimulationResetReady() const { return simulationRuntime_->ResetReady(); }
