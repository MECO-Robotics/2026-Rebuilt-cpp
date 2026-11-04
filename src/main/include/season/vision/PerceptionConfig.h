#pragma once

namespace season::vision::config
{
  // Object-detection camera subscription cadence.
  inline constexpr double kObjectDetectionPeriodSeconds = .005;

  // Nominal front and rear camera mounts in the robot frame.
  inline constexpr double kFrontCameraXMetres = .43;
  inline constexpr double kRearCameraXMetres = -.43;
  inline constexpr double kCameraHeightMetres = .54;
  inline constexpr double kCameraPitchDegrees = 15;
  inline constexpr double kRearCameraYawDegrees = 180;

  // Detection and clustering limits shared by camera IO and simulation.
  inline constexpr double kCameraRangeMetres = 6;
  inline constexpr double kClusterRadiusMetres = .15;
}
