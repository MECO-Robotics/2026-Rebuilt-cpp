#pragma once
#include "season/AutoDrive.h"
#include "season/Mechanisms.h"

namespace season {
// A timeout or unknown/inactive HUB must never authorize a hub shot.
inline bool OffensiveHubShotAllowed(std::optional<bool> hubActive,
                                    std::optional<frc::Pose2d> target,
                                    frc::Pose2d pose, frc::ChassisSpeeds speeds) {
  return hubActive.value_or(false) && target && AutoDriveArrived(pose,speeds,*target);
}
// Runs until interrupted. Possession is not sensed on this robot; collection
// and shooting are bounded attempts rather than exact capacity-based decisions.
frc2::CommandPtr OffensiveAutoDrive(AutoDrive& perception, base::Drive& drive,
                                    Mechanisms mechanisms, FieldGeometry field);
}
