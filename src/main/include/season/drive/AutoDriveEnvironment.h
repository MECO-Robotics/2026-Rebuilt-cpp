#pragma once
#include <base/AutoDrive.h>
#include <base/ObjectPerception.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace season {
// Simulation camera input and optional navigation policy supplied by the
// robot's runtime. The default camera reports no simulated detections.
class AutoDriveEnvironment {
public:
  virtual ~AutoDriveEnvironment() = default;
  virtual std::unique_ptr<base::ObjectDetectionIO> Camera(
      std::size_t index, const std::string &name, std::function<frc::Transform3d()> mount, units::second_t period);
  virtual bool TrackTarget() const { return false; }
  virtual std::optional<frc2::CommandPtr> NavigationCommand(
      base::Drive &, std::function<std::optional<frc::Pose2d>()>) { return std::nullopt; }
};
}
