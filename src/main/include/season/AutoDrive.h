#pragma once
#include <base/AutoDrive.h>
#include "season/MatchContext.h"
#include <base/ObjectPerception.h>
#include "season/FuelSimulation.h"
#ifndef __FRC_ROBORIO__
#include <meco/sim/SimulatedPieceSensor.h>
#endif

namespace season {
class AutoDrive {
 public:
  AutoDrive(base::Drive& drive, std::function<double()> intakeExtension,
      std::function<const MatchContext&()> matchContext);
  frc2::CommandPtr Command();
  void PublishMatchContext() const;
  // Same DriverStation-derived input on hardware and in simulation. Unknowns
  // stay explicit; this supplies timing context without selecting a strategy.
  const MatchContext& MatchState() const { return matchContext_(); }
#ifndef __FRC_ROBORIO__
  void UpdateSimulation(const frc::Pose3d& truth, const FuelModel& fuel, bool fieldColliders);
  void ResetSimulation();
#endif
 private:
  frc::Transform3d Mount(size_t camera) const;
  std::optional<frc::Pose2d> Target();
  void Status(const char* value);
  std::function<const MatchContext&()> matchContext_;
  base::Drive& drive_;
  std::function<double()> extension_;
  std::unique_ptr<base::ObjectPerception> perception_;
#ifndef __FRC_ROBORIO__
  std::array<std::unique_ptr<meco::sim::SimulatedPieceSensor>,2> sensors_;
  std::array<std::optional<base::ObjectDetectionFrame>,2> frames_;
  uint64_t sequence_{};
#endif
};
}
