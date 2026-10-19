#include "base/Drive.h"
#include "base/DriveTarget.h"
#include <frc/Timer.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <frc/DriverStation.h>
#include <ctre/phoenix6/Utils.hpp>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <frc/system/plant/DCMotor.h>
using namespace units::literals;
namespace p = ctre::phoenix6;
namespace s = p::swerve;
namespace base {
struct Drive::ControlRequest final : s::requests::SwerveRequest {
  DriveTarget target;
  s::requests::ApplyRobotSpeeds velocity;
  s::requests::SwerveDriveBrake brake;
  ControlRequest() {
    velocity.DriveRequestType = s::impl::DriveRequestType::Velocity;
    velocity.WheelForceFeedforwardsX.resize(4);
    velocity.WheelForceFeedforwardsY.resize(4);
  }
  ctre::phoenix::StatusCode Apply(const ControlParameters& params,
      std::span<std::unique_ptr<s::impl::SwerveModuleImpl> const> modules) override {
    const bool enabled = frc::DriverStation::IsEnabled();
    const auto value = target.Consume(frc::Timer::GetFPGATimestamp().value(), enabled);
    if (!enabled || (value.teleopOnly && !frc::DriverStation::IsTeleopEnabled())) {
      for (const auto& module : modules)
        module->Apply(p::controls::NeutralOut{}, p::controls::NeutralOut{});
      return ctre::phoenix::StatusCode::OK;
    }
    if (value.brake) return brake.Apply(params, modules);
    velocity.Speeds = DriveTarget::Resolve(value, params.currentPose.Rotation());
    for (int i=0;i<4;++i) {
      velocity.WheelForceFeedforwardsX[i] = units::newton_t{value.forceX[i]};
      velocity.WheelForceFeedforwardsY[i] = units::newton_t{value.forceY[i]};
    }
    return velocity.Apply(params, modules);
  }
};
Drive::Drive(DriveConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.odometryHz) || config_.odometryHz < 0)
    throw std::invalid_argument("Invalid odometry frequency");
  p::configs::TalonFXConfiguration drive, steer;
  drive.CurrentLimits.WithSupplyCurrentLimit(units::ampere_t{config_.driveCurrent}).WithSupplyCurrentLimitEnable(true);
  steer.CurrentLimits.WithStatorCurrentLimit(units::ampere_t{config_.steerCurrent}).WithStatorCurrentLimitEnable(true);
  auto factory = s::SwerveModuleConstantsFactory<p::configs::TalonFXConfiguration,p::configs::TalonFXConfiguration,p::configs::CANcoderConfiguration>{}
    .WithDriveMotorGearRatio(config_.driveRatio).WithSteerMotorGearRatio(config_.steerRatio)
    .WithCouplingGearRatio(config_.couplingRatio).WithWheelRadius(units::meter_t{config_.wheelRadius})
    .WithDriveMotorGains(p::configs::Slot0Configs{}.WithKP(config_.driveKP).WithKD(config_.driveKD).WithKS(config_.driveKS).WithKV(config_.driveKV))
    .WithSteerMotorGains(p::configs::Slot0Configs{}.WithKP(config_.steerKP).WithKD(config_.steerKD).WithKS(config_.steerKS).WithKV(config_.steerKV))
    .WithDriveMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage).WithSteerMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
    .WithDriveMotorType(s::DriveMotorArrangement::TalonFX_Integrated).WithSteerMotorType(s::SteerMotorArrangement::TalonFX_Integrated)
    .WithFeedbackSource(s::SteerFeedbackType::RemoteCANcoder)
    .WithSlipCurrent(units::ampere_t{config_.slipCurrent}).WithSpeedAt12Volts(units::meters_per_second_t{config_.maxSpeed})
    .WithDriveMotorInitialConfigs(drive).WithSteerMotorInitialConfigs(steer)
    .WithSteerInertia(units::kilogram_square_meter_t{.004}).WithDriveInertia(units::kilogram_square_meter_t{.025})
    .WithSteerFrictionVoltage(.1_V).WithDriveFrictionVoltage(.1_V);
  auto make = [&](int i) { const auto& m=config_.modules[i]; return factory.CreateModuleConstants(m.steer,m.drive,m.encoder,units::turn_t{m.offsetTurns},units::meter_t{m.xMeters},units::meter_t{m.yMeters},m.driveInverted,m.steerInverted,m.encoderInverted); };
  native_ = std::make_unique<NativeDrive>(s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon),units::hertz_t{config_.odometryHz},make(0),make(1),make(2),make(3));
  control_ = std::make_unique<ControlRequest>();
  native_->SetControl(*control_);
  SetName("Drive");
}
Drive::~Drive() { native_->GetOdometryThread().Stop(); native_.reset(); }
void Drive::Periodic() {
  if (frc::DriverStation::IsDisabled()) {
    auto a=frc::DriverStation::GetAlliance();
    native_->SetOperatorPerspectiveForward(frc::Rotation2d{a && *a==frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
    Stop();
  }
  const auto state=native_->GetState();
  frc::SmartDashboard::PutNumberArray("Drive/Pose",std::array<double,3>{state.Pose.X().value(),state.Pose.Y().value(),state.Pose.Rotation().Degrees().value()});
  frc::SmartDashboard::PutNumberArray("Drive/Speeds",std::array<double,3>{state.Speeds.vx.value(),state.Speeds.vy.value(),state.Speeds.omega.value()});
}
void Drive::SimulationPeriodic() { native_->UpdateSimState(.02_s,frc::RobotController::GetBatteryVoltage()); }
frc::Pose2d Drive::Pose() const { return native_->GetState().Pose; }
frc::ChassisSpeeds Drive::Speeds() const { return native_->GetState().Speeds; }
void Drive::ResetPose(const frc::Pose2d& pose) { native_->ResetPose(pose); }
void Drive::AddVision(const frc::Pose2d& pose,units::second_t time,std::array<double,3> noise) { native_->AddVisionMeasurement(pose,p::utils::FPGAToCurrentTime(time),noise); }
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds) {
  DriveTarget::Value value;
  value.speeds = speeds;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds,const pathplanner::DriveFeedforwards& ff) {
  DriveTarget::Value value;
  value.speeds = speeds;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  if (ff.robotRelativeForcesX.size() == 4 && ff.robotRelativeForcesY.size() == 4)
    for (int i=0;i<4;++i) {
      value.forceX[i] = ff.robotRelativeForcesX[i].value();
      value.forceY[i] = ff.robotRelativeForcesY[i].value();
    }
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::FieldDrive(double x,double y,double omega) {
  if (!frc::DriverStation::IsTeleopEnabled()) { Stop(); return; }
  auto alliance=frc::DriverStation::GetAlliance();
  double sign=alliance && *alliance==frc::DriverStation::Alliance::kRed ? -1 : 1;
  DriveTarget::Value value;
  value.speeds = {units::meters_per_second_t{x*sign}, units::meters_per_second_t{y*sign}, units::radians_per_second_t{omega}};
  value.fieldRelative = value.teleopOnly = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsTeleopEnabled(); });
}
void Drive::RunHeading(double x, double y, frc::Rotation2d heading, double kp, double maxOmega) {
  DriveTarget::Value value;
  value.speeds = {units::meters_per_second_t{x}, units::meters_per_second_t{y}, units::radians_per_second_t{0}};
  value.heading = heading;
  value.headingKP = kp;
  value.maxOmega = maxOmega;
  value.headingControl = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::Stop() { RunVelocity(frc::ChassisSpeeds{}); }
void Drive::StopWithX() {
  DriveTarget::Value value;
  value.brake = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::ConfigureAuto() {
  std::vector<frc::Translation2d> locations;
  for (const auto& m:config_.modules) locations.emplace_back(units::meter_t{m.xMeters},units::meter_t{m.yMeters});
  pathplanner::RobotConfig rc{units::kilogram_t{config_.massKg},units::kilogram_square_meter_t{config_.moi},
    pathplanner::ModuleConfig{units::meter_t{config_.wheelRadius},units::meters_per_second_t{config_.maxSpeed},config_.cof,frc::DCMotor::KrakenX60FOC(1).WithReduction(config_.driveRatio),units::ampere_t{config_.driveCurrent},1},locations};
  pathplanner::AutoBuilder::configure([this]{return Pose();},[this](const auto& pose){ResetPose(pose);},[this]{return Speeds();},
    [this](const frc::ChassisSpeeds& speed,const pathplanner::DriveFeedforwards& ff){RunVelocity(speed,ff);},
    std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants{5,0,0},pathplanner::PIDConstants{5,0,0}),rc,
    []{auto a=frc::DriverStation::GetAlliance(); return a && *a==frc::DriverStation::Alliance::kRed;},this);
}
static double Deadband(double value) { return std::abs(value)<.1 ? 0 : std::copysign((std::abs(value)-.1)/.9,value); }
frc2::CommandPtr Drive::Joystick(std::function<double()> x,std::function<double()> y,std::function<double()> omega) {
  return frc2::cmd::RunEnd([=,this]{FieldDrive(Deadband(x())*config_.maxSpeed,Deadband(y())*config_.maxSpeed,Deadband(omega())*config_.maxAngularRate);},[this]{Stop();},{this});
}
frc2::CommandPtr Drive::Aim(std::function<double()> x,std::function<double()> y,std::function<frc::Rotation2d()> heading) {
  return frc2::cmd::RunEnd([=,this]{
    if (!frc::DriverStation::IsTeleopEnabled()) { Stop(); return; }
    auto alliance=frc::DriverStation::GetAlliance();
    const double sign=alliance && *alliance==frc::DriverStation::Alliance::kRed ? -1 : 1;
    DriveTarget::Value value;
    value.speeds = {units::meters_per_second_t{Deadband(x())*config_.maxSpeed*sign},
                    units::meters_per_second_t{Deadband(y())*config_.maxSpeed*sign}, units::radians_per_second_t{0}};
    value.heading = heading();
    value.headingKP = 5;
    value.maxOmega = config_.maxAngularRate;
    value.headingControl = value.fieldRelative = value.teleopOnly = true;
    value.timestamp = frc::Timer::GetFPGATimestamp().value();
    control_->target.Publish(value, []{ return frc::DriverStation::IsTeleopEnabled(); });
  },[this]{Stop();},{this});
}
}
