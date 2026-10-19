#pragma once
#include <array>
#include <memory>
#include <mutex>
#include <frc/Notifier.h>
#include <string>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <functional>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <cstdint>
#include <units/time.h>
#include <pathplanner/lib/util/DriveFeedforwards.h>
namespace base {
class DriveTelemetry;
struct ModuleConfig {
  int drive, steer, encoder;
  double offsetTurns, xMeters, yMeters;
  bool driveInverted = false, steerInverted = false, encoderInverted = false;
};
struct DriveConfig {
  std::string bus = "rio";
  int pigeon = 0;
  // 0 selects Phoenix defaults: 250 Hz CAN FD, 100 Hz CAN 2.0.
  double odometryHz = 0;
  double driveRatio = 6.122448979591837, steerRatio = 150.0 / 7.0, couplingRatio = 0;
  double wheelRadius = 0.0508, maxSpeed = 4.572, maxAngularRate = 7;
  // Phoenix native swerve drive gains use rotor turns; Java used output-wheel turns.
  // Divide the Java P/D/V gains by driveRatio; static voltage is unchanged.
  double driveKP = 1 / 6.122448979591837, driveKD = 0, driveKS = .5, driveKV = .71 / 6.122448979591837;
  double steerKP = 25, steerKD = 0, steerKS = 0, steerKV = 2.5;
  double driveCurrent = 40, steerCurrent = 40, slipCurrent = 50;
  double massKg = 74.088, moi = 6.883, cof = 1.2;
  // 0 retains maxSpeed as the native speed-at-12V model value.
  double speedAt12Volts = 0, driveInertia = .025, steerInertia = .004;
  double autoTranslationKP = 5, autoRotationKP = 5;
  bool fusedEncoder = false, autoConfigFromGUI = false;
  std::array<ModuleConfig, 4> modules{{
    {31,41,21,0,.33655,.33655}, {32,42,22,0,.33655,-.33655},
    {33,43,23,0,-.33655,.33655}, {34,44,24,0,-.33655,-.33655}}};
};
class Drive : public frc2::SubsystemBase {
 public:
  explicit Drive(DriveConfig config = {});
  ~Drive() override;
  void Periodic() override;
  void SimulationPeriodic() override;
  frc::Pose2d Pose() const;
  frc::ChassisSpeeds Speeds() const;
  void ResetPose(const frc::Pose2d& pose);
  // Install during container initialization. All reset APIs run on robot thread;
  // the observer may realign external estimators but must not call ResetPose.
  void SetPoseResetCallback(std::function<void(const frc::Pose2d&)> callback);
  void AddVision(const frc::Pose2d& pose, units::second_t time, std::array<double,3> noise);
  void RunVelocity(const frc::ChassisSpeeds& speeds);
  void RunVelocity(const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& ff);
  void FieldDrive(double x, double y, double omega);
  // Robot-relative translation; heading feedback resolves on native control ticks.
  void RunHeading(double x, double y, frc::Rotation2d heading, double kp, double maxOmega);
  void Stop();
  void StopWithX();
  void ConfigureAuto();
  frc2::CommandPtr Joystick(std::function<double()> x, std::function<double()> y, std::function<double()> omega);
  frc2::CommandPtr Aim(std::function<double()> x, std::function<double()> y, std::function<frc::Rotation2d()> heading);
  const DriveConfig& Config() const { return config_; }
 private:
  struct NativeDrive;
  struct ControlRequest;
  DriveConfig config_;
  std::function<void(const frc::Pose2d&)> poseResetCallback_;
  struct Snapshot {
    frc::Pose2d pose;
    frc::ChassisSpeeds speeds;
    std::array<frc::SwerveModuleState,4> measured{}, targets{};
    int64_t timestampUs = 0;
    double odometryPeriod = 0;
    int failedDaqs = 0;
  };
  mutable std::mutex snapshotMutex_;
  Snapshot snapshot_;
  double nextTelemetry_ = 0, nextBusTelemetry_ = 0;
  std::unique_ptr<DriveTelemetry> telemetry_;
  std::unique_ptr<ControlRequest> control_;
  std::unique_ptr<NativeDrive> native_;
  std::unique_ptr<frc::Notifier> simNotifier_;
};
}
