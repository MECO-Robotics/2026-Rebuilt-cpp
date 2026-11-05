#pragma once

#include "season/drive/OffensiveAutoDrive.h"
#include "season/drive/ScoringPosition.h"
#include <base/AutoDrive.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

namespace season::offense
{
  // Drivetrain limits and collection routes used by offensive autonomous.
  base::AutoDriveConfig OffensiveMotionLimits(const base::DriveConfig &drive);
  std::shared_ptr<pathplanner::PathPlannerPath> CollectionSpline(
      frc::Pose2d start, const std::vector<base::ObjectCluster> &observations,
      frc::DriverStation::Alliance alliance, const FieldGeometry &field, const ScoringGrid &grid,
      bool active, bool ferry, int remaining, const base::AutoDriveConfig &limits);
}
