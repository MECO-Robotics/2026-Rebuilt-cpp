#pragma once
#include "season/drive/OffensiveConfig.h"
#include <base/AutoDrive.h>
#include <cmath>
#include <numbers>
#include <base/season2026/MatchContext.h>
#include <base/FootprintConfiguration.h>
#include <base/ObjectPerception.h>
#include <frc/geometry/Pose3d.h>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace season
{
  class FuelModel;
// Arrival requires both pose tolerance and low measured motion.
inline bool AutoDriveArrived(const frc::Pose2d &pose, const frc::ChassisSpeeds &speeds, const frc::Pose2d &goal)
{
  return pose.Translation().Distance(goal.Translation()).value() < offenseconfig::kArrivalDistanceMetres &&
         std::abs(std::remainder((goal.Rotation().Radians() - pose.Rotation().Radians()).value(),
                                 2 * std::numbers::pi)) < offenseconfig::kArrivalHeadingRadians &&
         std::hypot(speeds.vx.value(), speeds.vy.value()) < offenseconfig::kArrivalLinearSpeed && std::abs(speeds.omega.value()) < offenseconfig::kArrivalAngularSpeed;
}
// Bridges season perception and match state to reusable navigation.
class AutoDrive
{
public:
  AutoDrive(base::Drive &drive, std::function<double()> intakeExtension,
            std::function<const base::season2026::MatchContext &()> matchContext,
            units::second_t detectionUpdatePeriod = units::second_t{offenseconfig::kDetectionPeriodSeconds});
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
