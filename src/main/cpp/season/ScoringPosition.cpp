#include "season/ScoringPosition.h"
#include <frc/Filesystem.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace season {
bool RobotClearOfDepot(frc::Pose2d pose, frc::DriverStation::Alliance alliance, const FieldGeometry &field) {
  // The depot rails enclose a 27 by 42 inch area against the end wall. The
  // robot must be able to rotate while aiming, so use its corner radius.
  const double radius = std::hypot(.43, .43);
  constexpr double depth = 27. * .0254;
  constexpr double halfWidth = 42. * .0254 / 2.;
  const bool blue = alliance == frc::DriverStation::Alliance::kBlue;
  const double endDistance = blue ? pose.X().value() : field.length - pose.X().value();
  const double depotY = blue ? 6.066 : field.width - 6.066;
  const double dx = std::max(0., endDistance - depth);
  const double dy = std::max(0., std::abs(pose.Y().value() - depotY) - halfWidth);
  return std::hypot(dx, dy) > radius;
}

bool RobotClearOfBumps(frc::Pose2d pose) {
  const double x=pose.X().value(), y=pose.Y().value();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pose.Rotation().Radians().value()))
    return false;
  // Model plates from the 2026 field and conservatively surround the .86 m
  // square bumper with a circumscribed circle for every heading.
  const double halfRobot=std::hypot(.43,.43)+.10, halfRamp=44.4*.0254/2, halfWidth=73*.0254/2;
  for (auto side : {frc::DriverStation::Alliance::kBlue,frc::DriverStation::Alliance::kRed}) {
    const auto hub=HubPosition(side);
    for (double sign : {-1.,1.}) {
      const double bumpY=hub.Y().value()+sign*(47.+73.)*.0254/2;
      if (std::abs(x-hub.X().value()) <= halfRamp+halfRobot &&
          std::abs(y-bumpY) <= halfWidth+halfRobot)
        return false;
    }
  }
  return true;
}

bool HubScoringPositionValid(frc::Pose2d pose, frc::DriverStation::Alliance alliance,
                             const FieldGeometry &field, bool simulation) {
  const double x=pose.X().value(), y=pose.Y().value();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pose.Rotation().Radians().value()) ||
      x < .15 || y < .15 || x > field.length-.15 || y > field.width-.15)
    return false;
  // Center inside the alliance zone, with margin against localization error.
  const double boundary=AllianceBoundary(alliance,field);
  if (alliance==frc::DriverStation::Alliance::kBlue ? x > boundary-.15 : x < boundary+.15)
    return false;
  if (!RobotClearOfBumps(pose) || !RobotClearOfDepot(pose, alliance, field)) return false;
  return CalculateShot(pose,HubPosition(alliance),simulation).calibrated;
}

std::optional<ScoringGrid> LoadScoringGrid() {
  const auto file=wpi::MemoryBuffer::GetFile(frc::filesystem::GetDeployDirectory()+"/pathplanner/navgrid.json");
  if (!file) return std::nullopt;
  try {
    const auto json=wpi::json::parse(file.value()->GetCharBuffer());
    ScoringGrid result;
    result.cellSize=json.at("nodeSizeMeters").get<double>();
    result.blocked=json.at("grid").get<std::vector<std::vector<bool>>>();
    if (!std::isfinite(result.cellSize) || result.cellSize<=0 || result.blocked.empty() || result.blocked[0].empty())
      return std::nullopt;
    const auto width=result.blocked[0].size();
    for (const auto &row:result.blocked) if (row.size()!=width) return std::nullopt;
    return result;
  } catch (const std::exception &) { return std::nullopt; }
}

std::optional<frc::Pose2d> CheapestHubScoringNode(const ScoringGrid &grid, frc::Pose2d start,
    frc::DriverStation::Alliance alliance, const FieldGeometry &field, bool simulation) {
  const int rows=static_cast<int>(grid.blocked.size());
  if (!rows || !std::isfinite(grid.cellSize) || grid.cellSize<=0 || grid.blocked[0].empty() ||
      !std::isfinite(start.X().value()) || !std::isfinite(start.Y().value())) return std::nullopt;
  const int cols=static_cast<int>(grid.blocked[0].size());
  for (const auto &row:grid.blocked) if (static_cast<int>(row.size())!=cols) return std::nullopt;
  const int sx=static_cast<int>(std::floor(start.X().value()/grid.cellSize));
  const int sy=static_cast<int>(std::floor(start.Y().value()/grid.cellSize));
  auto free=[&](int x,int y){return x>=0 && y>=0 && x<cols && y<rows && !grid.blocked[y][x];};
  auto aimed=[&](frc::Translation2d position) {
    frc::Pose2d candidate{position,(HubPosition(alliance)-position).Angle()};
    for (int i=0;i<3;++i)
      candidate=frc::Pose2d{position,frc::Rotation2d{units::radian_t{CalculateShot(candidate,HubPosition(alliance),simulation).headingRadians}}};
    return candidate;
  };
  // The current point costs zero; do not move to a grid center merely because
  // the robot is already on level carpet in a valid shooting area.
  const auto current=aimed(start.Translation());
  if (HubScoringPositionValid(current,alliance,field,simulation)) return current;
  if (!free(sx,sy)) return std::nullopt;
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  std::vector<double> distance(rows*cols,std::numeric_limits<double>::infinity());
  distance[sy*cols+sx]=0; open.emplace(0,sy*cols+sx);
  while (!open.empty()) {
    const auto [cost,index]=open.top(); open.pop();
    if (cost>distance[index]) continue;
    const int x=index%cols, y=index/cols;
    const frc::Translation2d position{units::meter_t{(x+.5)*grid.cellSize},units::meter_t{(y+.5)*grid.cellSize}};
    const auto candidate=aimed(position);
    if (HubScoringPositionValid(candidate,alliance,field,simulation)) return candidate;
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) {
      if ((!dx && !dy) || !free(x+dx,y+dy) || (dx && dy && (!free(x+dx,y) || !free(x,y+dy)))) continue;
      const int next=(y+dy)*cols+x+dx;
      const double nextCost=cost+grid.cellSize*(dx && dy ? std::sqrt(2.) : 1.);
      if (nextCost<distance[next]) {distance[next]=nextCost;open.emplace(nextCost,next);}
    }
  }
  return std::nullopt;
}
}
