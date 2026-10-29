#pragma once
#include "season/RobotVisualization.h"
#include <frc/geometry/Transform3d.h>
namespace season {
// Front camera rides the hopper (model_4), not the downward-sloping intake rack.
inline frc::Transform3d HopperCameraMount(double extension, const frc::Transform3d& hopperToCamera) {
  const auto hopper=RemyComponentPoses(extension,0,0)[4];
  const auto camera=hopper.TransformBy(hopperToCamera);
  return {camera.Translation(),camera.Rotation()};
}
}
