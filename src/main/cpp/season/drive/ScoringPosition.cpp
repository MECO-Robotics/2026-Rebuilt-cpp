#include "season/drive/ScoringPosition.h"
#include <base/Replay.h>
#include <base/NavigationGrid.h>
#include <frc/Filesystem.h>
#include <algorithm>
#include <cmath>

namespace season
{
  namespace cfg = scoringpositionconfig;
  bool RobotClearOfDepot(frc::Pose2d pose, frc::DriverStation::Alliance alliance, const FieldGeometry &field)
  {
    // The depot rails enclose a 27 by 42 inch area against the end wall. The
    // robot must be able to rotate while aiming, so use its corner radius.
    const bool blue = alliance == frc::DriverStation::Alliance::kBlue;
    const double endDistance = blue ? pose.X().value() : field.length - pose.X().value();
    const double depotY = blue ? cfg::kDepotYMetres : field.width - cfg::kDepotYMetres;
    const double dx = std::max(0., endDistance - cfg::kDepotDepthMetres);
    const double dy = std::max(0., std::abs(pose.Y().value() - depotY) - cfg::kDepotHalfWidthMetres);
    return std::hypot(dx, dy) > cfg::kRobotCornerRadiusMetres;
  }

  bool RobotClearOfBumps(frc::Pose2d pose)
  {
    const double x = pose.X().value(), y = pose.Y().value();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pose.Rotation().Radians().value()))
      return false;
    // Model plates from the 2026 field and conservatively surround the .86 m
    // square bumper with a circumscribed circle for every heading.
    for (auto side : {frc::DriverStation::Alliance::kBlue, frc::DriverStation::Alliance::kRed})
    {
      const auto hub = HubPosition(side);
      for (double sign : {-1., 1.})
      {
        const double bumpY = hub.Y().value() + sign * cfg::kBumpCentreOffsetMetres;
        if (std::abs(x - hub.X().value()) <= cfg::kRampHalfLengthMetres + cfg::kRampRobotRadiusMetres &&
            std::abs(y - bumpY) <= cfg::kRampHalfWidthMetres + cfg::kRampRobotRadiusMetres)
          return false;
      }
    }
    return true;
  }

  bool HubScoringPositionValid(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                               const FieldGeometry &field, ShotCalibration calibration)
  {
    const double x = pose.X().value(), y = pose.Y().value();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pose.Rotation().Radians().value()) ||
        x < cfg::kScoringBoundaryMarginMetres || y < cfg::kScoringBoundaryMarginMetres || x > field.length - cfg::kScoringBoundaryMarginMetres || y > field.width - cfg::kScoringBoundaryMarginMetres)
      return false;
    // Center inside the alliance zone, with margin against localization error.
    const double boundary = AllianceBoundary(alliance, field);
    if (alliance == frc::DriverStation::Alliance::kBlue ? x > boundary - cfg::kScoringBoundaryMarginMetres : x < boundary + cfg::kScoringBoundaryMarginMetres)
      return false;
    if (!RobotClearOfBumps(pose) || !RobotClearOfDepot(pose, alliance, field))
      return false;
    return CalculateShot(pose, HubPosition(alliance), calibration).calibrated;
  }

  std::optional<ScoringGrid> LoadScoringGrid()
  {
    static bool loaded = false;
    static std::optional<ScoringGrid> cached;
    if (!loaded)
    {
      const auto path = frc::filesystem::GetDeployDirectory() + "/pathplanner/navgrid.json";
      cached = base::ProcessNavigationGridInput("Season/AutoDrive/NavigationGrid", path, 2 * 1024 * 1024);
      loaded = true;
    }
    return cached;
  }

  std::optional<frc::Pose2d> CheapestHubScoringNode(const ScoringGrid &grid, frc::Pose2d start,
                                                    frc::DriverStation::Alliance alliance, const FieldGeometry &field, ShotCalibration calibration)
  {
    const auto hub = HubPosition(alliance);
    auto aimed = [&](frc::Translation2d position)
    {
      frc::Pose2d candidate{position, (hub - position).Angle()};
      for (int i = 0; i < 3; ++i)
        candidate = frc::Pose2d{position, frc::Rotation2d{units::radian_t{CalculateShot(candidate, hub, calibration).headingRadians}}};
      return candidate;
    };
    const auto point = base::FindReachableGridPoint(grid, start.Translation(), [&](frc::Translation2d position)
    {
      return HubScoringPositionValid(aimed(position), alliance, field, calibration);
    });
    return point ? std::optional{aimed(*point)} : std::nullopt;
  }
}
