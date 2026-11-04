#pragma once

namespace season::vision::config
{
  // Object-detection camera subscription cadence.
  inline constexpr double kObjectDetectionPeriodSeconds = .005;

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
      .xMetres = .43, .yMetres = 0, .zMetres = .54,
      .rollDegrees = 0, .pitchDegrees = 15, .yawDegrees = 0};
  inline constexpr CameraMount kRearCamera{
      .xMetres = -.43, .yMetres = 0, .zMetres = .54,
      .rollDegrees = 0, .pitchDegrees = 15, .yawDegrees = 180};

  // Maximum range of the simulated object cameras; physical detection is set by the camera pipeline.
  inline constexpr double kCameraRangeMetres = 6;
  // Distance within which observations are combined into an object cluster.
  inline constexpr double kClusterRadiusMetres = .15;
}
