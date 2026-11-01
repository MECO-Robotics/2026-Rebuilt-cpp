#pragma once
#include "season/shooting/Shooting.h"
#include <optional>
#include <vector>

namespace season {
struct ScoringGrid {
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
