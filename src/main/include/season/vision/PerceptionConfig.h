#pragma once
#include "season/RobotGeometry.h"

namespace season::vision::config
{
  inline constexpr double kCameraForwardOffsetInches = 17.0;
  inline constexpr double kCameraHeightInches = 21.25;
  // Object-detection camera subscription cadence.
  inline constexpr double kObjectDetectionPeriodSeconds = 1.0 / 200.0;

  // Camera positions are metres and angles are degrees: robot +X is forward,
  // +Y left, and +Z up. The front mount is relative to the moving hopper;
  // its frame shares the robot origin when retracted and translates forward
  // without rotating as the rack extends. The rear mount is relative to the robot frame.
  struct CameraMount
  {
    double xMetres, yMetres, zMetres;
    double rollDegrees, pitchDegrees, yawDegrees;
  };
  inline constexpr CameraMount kFrontCamera{
      .xMetres = kCameraForwardOffsetInches * season::kMetresPerInch, .yMetres = 0,
      .zMetres = kCameraHeightInches * season::kMetresPerInch,
      .rollDegrees = 0, .pitchDegrees = 15, .yawDegrees = 0};
  inline constexpr CameraMount kRearCamera{
      .xMetres = -kCameraForwardOffsetInches * season::kMetresPerInch, .yMetres = 0,
      .zMetres = kCameraHeightInches * season::kMetresPerInch,
      .rollDegrees = 0, .pitchDegrees = 15, .yawDegrees = 180};

  // Distance within which observations are combined into an object cluster.
  inline constexpr double kClusterRadiusMetres = .15;
}
