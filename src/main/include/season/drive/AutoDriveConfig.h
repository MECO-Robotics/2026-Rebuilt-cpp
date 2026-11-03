#pragma once
#include <cmath>
namespace season::autodriveconfig
{
  inline constexpr double kAutoSpinUpTimeoutSeconds = 2.;
  inline constexpr double kChoreoTranslationP = 10, kChoreoRotationP = 7;
  inline constexpr double kCameraFrontXMetres = .43, kCameraRearXMetres = -.43, kCameraHeightMetres = .54;
  inline constexpr double kCameraPitchDegrees = 15, kRearCameraYawDegrees = 180;
  inline constexpr double kCameraRangeMetres = 6, kClusterRadiusMetres = .15;
  inline constexpr double kDefaultTargetXMetres = 3., kDefaultTargetYMetres = .65;
  inline constexpr int kMaxGridBytes = 262144;
  inline constexpr double kTargetReplanDistanceMetres = .3;
  inline constexpr double kFootprintLengthMetres = .86, kFootprintWidthMetres = .86;
  inline constexpr double kHeadingToleranceRadians = .02, kAngularSettleSpeed = .02;
  inline constexpr int kFinalSettleSamples = 50;
  inline constexpr double kPoseAngularDamping = .5, kAngularStopTolerance = .01;
  inline constexpr double kTranslationStallTimeoutSeconds = 3, kTranslationStallRequest = .01;
  inline constexpr double kMaxAcceleration = 4, kCollisionBottomThresholdMetres = .05, kCollisionTopThresholdMetres = .45;
  inline constexpr double kPostCommandWaitSeconds = .3;
  inline constexpr double kDepotDepthInches = 27., kDepotWidthInches = 42., kDepotYMetres = 6.066;
  inline constexpr double kRobotHalfMetres = .43, kRampRobotMarginMetres = .10;
  inline constexpr double kRampLengthInches = 44.4, kRampWidthInches = 73., kHubWidthInches = 47.;
  inline constexpr double kDepotDepthMetres = kDepotDepthInches * .0254;
  inline constexpr double kDepotHalfWidthMetres = kDepotWidthInches * .0254 / 2;
  inline const double kRobotCornerRadiusMetres = std::hypot(kRobotHalfMetres, kRobotHalfMetres);
  inline const double kRampRobotRadiusMetres = kRobotCornerRadiusMetres + kRampRobotMarginMetres;
  inline constexpr double kRampHalfLengthMetres = kRampLengthInches * .0254 / 2;
  inline constexpr double kRampHalfWidthMetres = kRampWidthInches * .0254 / 2;
  inline constexpr double kBumpCentreOffsetMetres = (kHubWidthInches + kRampWidthInches) * .0254 / 2;
  inline constexpr double kScoringBoundaryMarginMetres = .15;
}
