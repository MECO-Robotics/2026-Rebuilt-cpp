#pragma once
#include "season/shooting/Shooting.h"
#include <cmath>
#include <optional>
#include <vector>

namespace season
{
  namespace scoringpositionconfig
  {
    inline constexpr double kDepotDepthInches = 27., kDepotWidthInches = 42., kDepotYMetres = 6.066;
    inline constexpr double kRobotHalfMetres = .43, kRampRobotMarginMetres = .10;
    inline constexpr double kRampLengthInches = 44.4, kRampWidthInches = 73., kHubWidthInches = 47.;
    inline constexpr double kDepotDepthMetres = kDepotDepthInches * .0254;
    inline constexpr double kDepotHalfWidthMetres = kDepotWidthInches * .0254 / 2;
    inline const double kRobotCornerRadiusMetres = std::hypot(kRobotHalfMetres, kRobotHalfMetres);
    inline const double kRampRobotRadiusMetres = kRobotCornerRadiusMetres + kRampRobotMarginMetres;
    inline constexpr double kRampHalfLengthMetres = kRampLengthInches * .0254 / 2;
    inline constexpr double kRampHalfWidthMetres = kRampWidthInches * .0254 / 2;
    inline constexpr double kBumpCentreOffsetMetres = (kHubWidthInches + kRampWidthInches) * .0254 / 2;
    inline constexpr double kScoringBoundaryMarginMetres = .15;
  }

  struct ScoringGrid
  {
    double cellSize = 0;
    std::vector<std::vector<bool>> blocked;
  };
  // Static-grid path distance; diagonal moves cannot cut blocked corners.
  std::optional<frc::Pose2d> CheapestHubScoringNode(const ScoringGrid &grid, frc::Pose2d start,
                                                    frc::DriverStation::Alliance alliance, const FieldGeometry &field, bool simulation);
  std::optional<ScoringGrid> LoadScoringGrid();
  bool RobotClearOfBumps(frc::Pose2d pose);
  bool RobotClearOfDepot(frc::Pose2d pose, frc::DriverStation::Alliance alliance, const FieldGeometry &field);
  bool HubScoringPositionValid(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                               const FieldGeometry &field, bool simulation);
}
