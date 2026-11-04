#pragma once
#include <base/AutoDrive.h>
#include <cmath>
#include <numbers>
#include <base/season2026/MatchContext.h>
#include <base/FootprintConfiguration.h>
#include <base/ObjectPerception.h>
#include "season/vision/PerceptionConfig.h"
#include "season/RobotGeometry.h"
#include <frc/geometry/Pose3d.h>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace season
{
namespace autodriveconfig
{
  // Dashboard target coordinates published at startup and used as fallbacks, in metres.
  inline constexpr double kDefaultTargetXMetres = 3., kDefaultTargetYMetres = .65;
  // Maximum size of the configured-footprint JSON file accepted at startup, in bytes.
  inline constexpr int kMaxGridBytes = 262144;
  // Target movement that triggers replanning in a tracking footprint mode, in metres.
  inline constexpr double kTargetReplanDistanceMetres = .3;
  // Robot footprint dimensions used by the simulated footprint follower, in metres.
  inline constexpr double kFootprintLengthMetres = geometry::kLengthMetres,
                          kFootprintWidthMetres = geometry::kWidthMetres;
  // The following follower settings apply to simulated footprint modes.
  // Heading error allowed when settling at a navigation target, in radians.
  inline constexpr double kHeadingToleranceRadians = .02;
  // Maximum angular speed considered settled, in radians per second.
  inline constexpr double kAngularSettleSpeed = .02;
  // Consecutive settled samples required before reporting arrival.
  inline constexpr int kFinalSettleSamples = 50;
  // Intermediate settled samples used by the simulated footprint follower.
  inline constexpr int kIntermediateSettleSamples = 10;
  // Damping applied to angular correction while approaching a target.
  inline constexpr double kPoseAngularDamping = .5;
  // Angular velocity below which the controller treats rotation as stopped, in radians per second.
  inline constexpr double kAngularStopTolerance = .01;
  // Time without translational progress before navigation declares a stall, in seconds.
  inline constexpr double kTranslationStallTimeoutSeconds = 3;
  // Minimum requested translation speed used to distinguish a stalled command, in metres per second.
  inline constexpr double kTranslationStallRequest = .01;
  // Maximum acceleration used by the optional simulated terrain profile, in metres per second squared.
  inline constexpr double kMaxAcceleration = 4;
  // Lower vertical bound for obstacles considered in collision checks, in metres.
  inline constexpr double kCollisionBottomThresholdMetres = .05;
  // Upper vertical bound for obstacles considered in collision checks, in metres.
  inline constexpr double kCollisionTopThresholdMetres = .45;
  // Status and arrival checks apply to both simulated and physical navigation.
  // Delay after a navigation command before publishing its final status, in seconds.
  inline constexpr double kPostCommandWaitSeconds = .3;
  // Maximum pose error and measured motion accepted by the arrival check, in metres, radians, and per-second units.
  inline constexpr double kArrivalDistanceMetres = .05, kArrivalHeadingRadians = .06,
                         kArrivalLinearSpeed = .2, kArrivalAngularSpeed = .2;
}

  class FuelModel;
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
            std::function<const base::season2026::MatchContext &()> matchContext,
            units::second_t detectionUpdatePeriod = units::second_t{vision::config::kObjectDetectionPeriodSeconds});
  ~AutoDrive();
  // Create a fresh drivetrain-owning navigation command.
  frc2::CommandPtr Command();
  // Fresh, estimated field-frame observations from the camera perception path.
  // Reference is valid until perception is updated or cleared.
  const std::vector<base::ObjectCluster> &VisibleFuel() const;
  void PublishMatchContext() const;
  // Same DriverStation-derived input on hardware and in simulation. Unknowns
  // stay explicit; this supplies timing context without selecting a strategy.
  const base::season2026::MatchContext &MatchState() const { return matchContext_(); }
#ifndef __FRC_ROBORIO__
  void UpdateSimulation(const frc::Pose3d &truth, const FuelModel &fuel, bool fieldColliders);
  void ResetSimulation();
#endif
private:
  frc::Transform3d Mount(size_t camera) const;
  // Select a dashboard pose or a fresh perceived fuel target.
  std::optional<frc::Pose2d> Target();
  void Status(const char *value);
  std::function<const base::season2026::MatchContext &()> matchContext_;
  base::Drive &drive_;
  std::function<double()> extension_;
  std::unique_ptr<base::ObjectPerception> perception_;
  std::optional<base::FootprintConfiguration> configuredFootprint_;
#ifndef __FRC_ROBORIO__
  struct SimulationSensors;
  std::unique_ptr<SimulationSensors> simulationSensors_;
#endif
};
} // namespace season
