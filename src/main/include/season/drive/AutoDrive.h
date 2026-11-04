#pragma once
#include <base/AutoDrive.h>
#include <cmath>
#include <numbers>
#include <base/season2026/MatchContext.h>
#include <base/FootprintConfiguration.h>
#include <base/ObjectPerception.h>
#include "season/vision/PerceptionConfig.h"
#include <frc/geometry/Pose3d.h>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace season
{
namespace autodriveconfig
{
  inline constexpr double kDefaultTargetXMetres = 3., kDefaultTargetYMetres = .65;
  inline constexpr int kMaxGridBytes = 262144;
  inline constexpr double kTargetReplanDistanceMetres = .3;
  inline constexpr double kFootprintLengthMetres = .86, kFootprintWidthMetres = .86;
  inline constexpr double kHeadingToleranceRadians = .02, kAngularSettleSpeed = .02;
  inline constexpr int kFinalSettleSamples = 50;
  inline constexpr double kPoseAngularDamping = .5, kAngularStopTolerance = .01;
  inline constexpr double kTranslationStallTimeoutSeconds = 3, kTranslationStallRequest = .01;
  inline constexpr double kMaxAcceleration = 4, kCollisionBottomThresholdMetres = .05, kCollisionTopThresholdMetres = .45;
  inline constexpr double kPostCommandWaitSeconds = .3;
  inline constexpr double kArrivalDistanceMetres = .05, kArrivalHeadingRadians = .06;
  inline constexpr double kArrivalLinearSpeed = .2, kArrivalAngularSpeed = .2;
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
