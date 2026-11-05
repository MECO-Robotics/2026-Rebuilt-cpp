#pragma once
#ifndef __FRC_ROBORIO__
#include "season/drive/AutoDriveEnvironment.h"
#include <meco/sim/SimulatedPieceCameraBank.h>
#include <array>
#include <cstdint>
namespace season { class FuelModel; }
namespace season::simulation {
// Owns desktop-only camera models and optional simulation navigation policy.
class AutoDriveSimulation final : public AutoDriveEnvironment {
public:
  AutoDriveSimulation();
  std::unique_ptr<base::ObjectDetectionIO> Camera(std::size_t, const std::string &, std::function<frc::Transform3d()>, units::second_t) override;
  bool TrackTarget() const override;
  std::optional<frc2::CommandPtr> NavigationCommand(base::Drive &, std::function<std::optional<frc::Pose2d>()>) override;
  void Update(const frc::Pose3d &, const FuelModel &, bool, const std::vector<base::ObjectCluster> &, const base::Drive &);
  void Reset();
private:
  meco::sim::SimulatedPieceCameraBank cameras_{2};
};
}
#endif
