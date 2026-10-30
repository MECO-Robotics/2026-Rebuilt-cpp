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
enum class OffensiveDecision { Wait, Collect, Score, Ferry };
// Upstream single-robot arbitration adapted to an estimated, not measured, load.
inline OffensiveDecision DecideOffense(std::optional<bool> active,
    std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d pose,
    const FieldGeometry& field, const std::vector<base::ObjectCluster>& points,
    int estimatedFuel, double capacity, bool scoreCommitted=false, bool ferryCommitted=false,
    bool autonomous=false, std::optional<double> periodTime=std::nullopt, bool collectionExpired=false,
    bool loadUnknown=true) {
  if (!active || !alliance) return OffensiveDecision::Wait;
  bool anyEligible=false, friendlyFuel=false, nearbyMidfield=false;
  const bool inZone=InAllianceZone(*alliance,pose.Translation(),field);
  const bool capacityKnown=std::isfinite(capacity) && capacity>0;
  const bool full=capacityKnown && estimatedFuel>=capacity;
  const double remaining=capacityKnown ? std::clamp(1.-estimatedFuel/capacity,0.,1.) : 1.;
  for (const auto& p:points) {
    const bool friendly=InAllianceZone(*alliance,p.position,field);
    if (friendly) friendlyFuel=true;
    if (*active || !friendly) anyEligible=true;
    if (!friendly && p.position.Distance(pose.Translation()).value()<=2.+1.5*remaining)
      nearbyMidfield=true;
  }
  const bool autoReturn=autonomous && periodTime && std::isfinite(*periodTime) && *periodTime<=13.;
  // No possession sensor: zero inferred pickups is unknown, not proof of an
  // empty hopper. Timed attempts may therefore operate on an empty hopper.
  const bool mayCarry=estimatedFuel>0 || loadUnknown;
  if (mayCarry && *active && (autoReturn || collectionExpired ||
      (estimatedFuel>0 && (scoreCommitted || full ||
        (inZone && !friendlyFuel) || (!inZone && !nearbyMidfield))))) return OffensiveDecision::Score;
  if (mayCarry && !*active && (collectionExpired ||
      (estimatedFuel>0 && (ferryCommitted || full || !anyEligible)))) return OffensiveDecision::Ferry;
  return OffensiveDecision::Collect;
}
// Runs until interrupted. Possession is not sensed on this robot; collection
// and shooting are bounded attempts rather than exact capacity-based decisions.
frc2::CommandPtr OffensiveAutoDrive(AutoDrive& perception, base::Drive& drive,
                                    Mechanisms mechanisms, FieldGeometry field);
}
