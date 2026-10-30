#pragma once
#include <base/AutoDrive.h>
#include <cmath>
#include <numbers>
#include <base/season2026/MatchContext.h>
#include <base/FootprintConfiguration.h>
#include <base/ObjectPerception.h>
#include "season/Simulation.h"
#ifndef __FRC_ROBORIO__
#include <meco/sim/SimulatedPieceSensor.h>
#endif

namespace season
{
  inline bool AutoDriveArrived(const frc::Pose2d &pose, const frc::ChassisSpeeds &speeds, const frc::Pose2d &goal)
  {
    return pose.Translation().Distance(goal.Translation()).value() < .05 &&
           std::abs(std::remainder((goal.Rotation().Radians() - pose.Rotation().Radians()).value(), 2 * std::numbers::pi)) < .06 &&
           std::hypot(speeds.vx.value(), speeds.vy.value()) < .2 && std::abs(speeds.omega.value()) < .2;
  }
  class AutoDrive
  {
  public:
    AutoDrive(base::Drive &drive, std::function<double()> intakeExtension,
              std::function<const base::season2026::MatchContext &()> matchContext);
    frc2::CommandPtr Command();
    // Fresh, estimated field-frame observations from the camera perception path.
    std::vector<frc::Translation2d> VisibleFuel() const;
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
    std::optional<frc::Pose2d> Target();
    void Status(const char *value);
    std::function<const base::season2026::MatchContext &()> matchContext_;
    base::Drive &drive_;
    std::function<double()> extension_;
    std::unique_ptr<base::ObjectPerception> perception_;
    std::optional<base::FootprintConfiguration> configuredFootprint_;
#ifndef __FRC_ROBORIO__
    std::array<std::unique_ptr<meco::sim::SimulatedPieceSensor>, 2> sensors_;
    std::array<std::optional<base::ObjectDetectionFrame>, 2> frames_;
    uint64_t sequence_{};
#endif
  };
}
