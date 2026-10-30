#pragma once
#include "season/AutoDrive.h"
#include "season/Mechanisms.h"
#include <algorithm>

namespace season {
// A timeout or unknown/inactive HUB must never authorize a hub shot.
inline bool OffensiveHubShotAllowed(std::optional<bool> hubActive,
                                    std::optional<frc::Pose2d> target,
                                    frc::Pose2d pose, frc::ChassisSpeeds speeds) {
  return hubActive.value_or(false) && target && AutoDriveArrived(pose,speeds,*target);
}
// Anonymous detections can disappear due to pickup or occlusion. This is a
// heuristic, not a measured hopper count.
class OffensiveFuelEstimate {
 public:
  void Observe(const std::vector<base::ObjectCluster>& points, frc::Pose2d pose) {
    for (const auto& old:previous_) {
      const auto local=(old.position-pose.Translation()).RotateBy(-pose.Rotation());
      const bool mouth=local.X().value()>=.20 && local.X().value()<=.70 && std::abs(local.Y().value())<=.4;
      const bool seen=std::any_of(points.begin(),points.end(),[&](const auto& p) {return p.position.Distance(old.position).value()<.12;});
      if (mouth && !seen) ++count_;
    }
    previous_=points;
  }
  void StartCollection() {previous_.clear();}
  void CompleteFeed(bool fed) {if (fed) count_=0;}
  int Count() const {return count_;}
 private:
  int count_=0;
  std::vector<base::ObjectCluster> previous_;
};
// Runs until interrupted. Possession is not sensed on this robot; collection
// and shooting are bounded attempts rather than exact capacity-based decisions.
frc2::CommandPtr OffensiveAutoDrive(AutoDrive& perception, base::Drive& drive,
                                    Mechanisms mechanisms, FieldGeometry field);
}
