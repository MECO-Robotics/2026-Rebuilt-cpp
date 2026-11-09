#pragma once
#include <base/AutoDrive.h>
#include <cmath>
#include <numbers>
#include "season/runtime/MatchContext.h"
#include <base/FootprintConfiguration.h>
#include <base/ObjectPerception.h>
#include "season/vision/PerceptionConfig.h"
#include "season/RobotGeometry.h"
#include <frc/geometry/Pose3d.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace season
{
// Simulation camera input and optional navigation policy supplied by the
// robot's runtime. The default camera reports no simulated detections.
class AutoDriveEnvironment {
public:
  virtual ~AutoDriveEnvironment() = default;
  virtual std::unique_ptr<base::ObjectDetectionIO> Camera(
      std::size_t index, const std::string &name, std::function<frc::Transform3d()> mount, units::second_t period);
  virtual bool TrackTarget() const { return false; }
  virtual std::optional<frc2::CommandPtr> NavigationCommand(
      base::Drive &, std::function<std::optional<frc::Pose2d>()>) { return std::nullopt; }
};

namespace autodriveconfig
{
  // Dashboard target coordinates published at startup and used as fallbacks, in metres.
  inline constexpr double kDefaultTargetXMetres = 3., kDefaultTargetYMetres = .65;
  // Maximum size of the configured-footprint JSON file accepted at startup, in bytes.
  inline constexpr int kMaxFootprintConfigBytes = 262144;
  // Target movement that triggers replanning in a tracking footprint mode, in metres.
  inline constexpr double kTargetReplanDistanceMetres = .3;
  // Status and arrival checks apply to both simulated and physical navigation.
  // Delay after a navigation command before publishing its final status, in seconds.
  inline constexpr double kPostCommandWaitSeconds = .3;
  // Maximum pose error and measured motion accepted by the arrival check, in metres, radians, and per-second units.
  inline constexpr double kArrivalDistanceMetres = .05, kArrivalHeadingRadians = .06,
                         kArrivalLinearSpeed = .2, kArrivalAngularSpeed = .2;
}

// Arrival requires both pose tolerance and low measured motion.
inline bool AutoDriveArrived(const frc::Pose2d &pose, const frc::ChassisSpeeds &speeds, const frc::Pose2d &goal)
{
  return pose.Translation().Distance(goal.Translation()).value() < autodriveconfig::kArrivalDistanceMetres &&
         std::abs(std::remainder((goal.Rotation().Radians() - pose.Rotation().Radians()).value(),
                                 2 * std::numbers::pi)) < autodriveconfig::kArrivalHeadingRadians &&
         std::hypot(speeds.vx.value(), speeds.vy.value()) < autodriveconfig::kArrivalLinearSpeed && std::abs(speeds.omega.value()) < autodriveconfig::kArrivalAngularSpeed;
}
// Bridges season perception and match state to reusable navigation.
class AutoDrive
{
public:
  AutoDrive(base::Drive &drive, std::function<double()> intakeExtension,
            std::function<const season::runtime::MatchContext &()> matchContext,
            units::second_t detectionUpdatePeriod = units::second_t{vision::config::kObjectDetectionPeriodSeconds},
            std::shared_ptr<AutoDriveEnvironment> environment = {});
  ~AutoDrive();
  // Create a fresh drivetrain-owning navigation command.
  frc2::CommandPtr Command();
  // Fresh, estimated field-frame observations from the camera perception path.
  // Reference is valid until perception is updated or cleared.
  const std::vector<base::ObjectCluster> &VisibleFuel() const;
  void PublishMatchContext() const;
  // Same DriverStation-derived input on hardware and in simulation. Unknowns
  // stay explicit; this supplies timing context without selecting a strategy.
  const season::runtime::MatchContext &MatchState() const { return matchContext_(); }
  // Discard cached observations after a source reset.
  void ClearPerception();
private:
  frc::Transform3d Mount(size_t camera) const;
  void LoadConfiguredFootprint();
  // Select a dashboard pose or a fresh perceived fuel target.
  std::optional<frc::Pose2d> Target();
  void Status(const char *value);
  std::function<const season::runtime::MatchContext &()> matchContext_;
  base::Drive &drive_;
  std::function<double()> extension_;
  std::unique_ptr<base::ObjectPerception> perception_;
  std::optional<base::FootprintConfiguration> configuredFootprint_;
  bool configuredFootprintLoaded_{};
  std::shared_ptr<AutoDriveEnvironment> environment_;
};
} // namespace season
