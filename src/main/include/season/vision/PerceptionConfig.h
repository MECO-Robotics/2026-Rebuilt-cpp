#pragma once

namespace season::vision::config
{
  // Object-detection camera subscription cadence.
  inline constexpr double kObjectDetectionPeriodSeconds = .005;

  // Object-detection camera mounting positions and orientations in the robot frame.
  inline constexpr double kFrontCameraXMetres = .43, kRearCameraXMetres = -.43,
                          kCameraHeightMetres = .54, kCameraPitchDegrees = 15,
                          kRearCameraYawDegrees = 180;

  // Maximum camera observation range and clustering distance for object detections.
  inline constexpr double kCameraRangeMetres = 6, kClusterRadiusMetres = .15;
}
