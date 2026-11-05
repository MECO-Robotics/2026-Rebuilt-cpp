#pragma once
#include "season/shooting/Shooting.h"
#include "season/RobotGeometry.h"
#include "base/NavigationGrid.h"
#include <cmath>
#include <optional>
#include <vector>

namespace season
{
  namespace scoringpositionconfig
  {
    // Depot dimensions from the field geometry, in inches.
    inline constexpr double kDepotDepthInches = 27., kDepotWidthInches = 42.;
    // Field Y coordinate of the depot centreline, in metres.
    inline constexpr double kDepotYMetres = 6.066;
    // Extra clearance around the robot when checking ramp regions, in metres.
    inline constexpr double kRampRobotMarginMetres = .10;
    // Ramp dimensions from the field geometry, in inches.
    inline constexpr double kRampLengthInches = 44.4, kRampWidthInches = 73.;
    // Convert depot depth to metres for field-coordinate comparisons.
    inline constexpr double kDepotDepthMetres = kDepotDepthInches * .0254;
    // Half the depot width converted to metres for side-boundary checks.
    inline constexpr double kDepotHalfWidthMetres = kDepotWidthInches * .0254 / 2;
    // Distance from the robot centre to a footprint corner, in metres.
    inline const double kRobotCornerRadiusMetres = std::hypot(geometry::kLengthMetres / 2, geometry::kWidthMetres / 2);
    // Expanded footprint radius used to keep the robot clear of ramp edges, in metres.
    inline const double kRampRobotRadiusMetres = kRobotCornerRadiusMetres + kRampRobotMarginMetres;
    // Half ramp dimensions converted to metres for centre-based geometry checks.
    inline constexpr double kRampHalfLengthMetres = kRampLengthInches * .0254 / 2,
                           kRampHalfWidthMetres = kRampWidthInches * .0254 / 2;
    // Centre offset from the hub to the bump/ramp centreline, in metres.
    inline constexpr double kBumpCentreOffsetMetres = (season::kHubWidthInches + kRampWidthInches) * .0254 / 2;
    // Extra inset from field scoring boundaries required for a valid position, in metres.
    inline constexpr double kScoringBoundaryMarginMetres = .15;
  }

  using ScoringGrid = base::NavigationGrid;
  // Static-grid path distance; diagonal moves cannot cut blocked corners.
  std::optional<frc::Pose2d> CheapestHubScoringNode(const ScoringGrid &grid, frc::Pose2d start,
                                                    frc::DriverStation::Alliance alliance, const FieldGeometry &field, ShotCalibration calibration);
  std::optional<ScoringGrid> LoadScoringGrid();
  bool RobotClearOfBumps(frc::Pose2d pose);
  bool RobotClearOfDepot(frc::Pose2d pose, frc::DriverStation::Alliance alliance, const FieldGeometry &field);
  bool HubScoringPositionValid(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                               const FieldGeometry &field, ShotCalibration calibration);
}
