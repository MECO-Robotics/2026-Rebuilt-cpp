#pragma once
#include "season/shooting/ShootingConfig.h"
#include <array>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Pure season algorithms. Distances are meters, angles radians, mechanism
// positions rotations, velocities rotations/second; time is monotonic seconds.
namespace season
{
  enum class ShotMode
  {
    Hub,
    Ferry
  };
  // Target position is expressed in the blue-origin field frame.
  struct ShotTarget
  {
    ShotMode mode;
    frc::Translation2d position;
  };
  // Hood uses output rotations; flywheel speed uses rotations per second.
  struct ShotSolution
  {
    double distanceMeters, headingRadians, hoodRotations, flywheelRps;
    bool calibrated;
  };
  // Safety includes field clearance and the configured shot limits.
  struct FerryPlan
  {
    double hoodRotations, flywheelRps;
    bool safe;
  };
  // Adapter must populate these from the official 2026 AprilTagFieldLayout;
  // trench references are midpoint(tag22,23) for blue and midpoint(tag6,7) for red.
  struct FieldGeometry
  {
    double length, width;
    frc::Translation2d blueTrenchReference, redTrenchReference;
  };
  struct ShotTuning
  {
    double ferryHoodRotations = kDefaultFerryHoodRotations, ferryFlywheelRps = kDefaultFerryFlywheelRps;
  };
  frc::Translation2d HubPosition(frc::DriverStation::Alliance alliance);
  double AllianceBoundary(frc::DriverStation::Alliance alliance, const FieldGeometry &field);
  bool InAllianceZone(frc::DriverStation::Alliance alliance, frc::Translation2d point, const FieldGeometry &field);
  bool SeesAllianceSideTag(frc::DriverStation::Alliance alliance, const std::vector<int> &ids);
  ShotTarget FerryTarget(frc::DriverStation::Alliance alliance, frc::Pose2d robot, const FieldGeometry &field);
  ShotTarget DriverAimTarget(frc::DriverStation::Alliance alliance, frc::Pose2d robot, const FieldGeometry &field);
  ShotTarget SelectTarget(frc::DriverStation::Alliance alliance, frc::Pose2d robot, const std::vector<int> &ids,
                          const FieldGeometry &field);
  double FerryFlywheelRps(double distanceMeters, double hoodRotations);
  FerryPlan CalculateFerry(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field, ShotTuning tuning = {});
  ShotSolution CalculateShot(frc::Pose2d robot, frc::Translation2d target, bool simulation = false);
  ShotSolution CalculateShot(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field, bool simulation = false,
                             ShotTuning tuning = {});
  // Speeds are measured robot-relative. Returns an invalid solution when no
  // bounded-velocity trajectory clears the field and lands in the alliance zone.
  ShotSolution CalculateMovingFerry(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field,
                                    frc::ChassisSpeeds robotSpeeds, bool simulation = false, ShotTuning tuning = {});
  ShotSolution CalculateDriverAim(frc::DriverStation::Alliance alliance, frc::Pose2d robot, const FieldGeometry &field,
                                  bool simulation = false);
  // Readiness explains why coordinated feeding is permitted or blocked.
  enum class Readiness
  {
    PoseNotReady,
    OutOfRange,
    UnsafeFerryPath,
    MovingTooFast,
    Aiming,
    HoodMoving,
    FlywheelSpinningUp,
    CalibrationMode,
    Ready,
    ForceFeedOverride
  };
  const char *ReadinessName(Readiness value);
  Readiness EvaluateReadiness(bool poseReady, ShotSolution solution, double vx, double vy, double omega, double heading,
                              double hood, double flywheel, double maxTranslation = kDefaultShotMaxTranslation,
                              double maxOmega = kDefaultShotMaxOmega);
  double ClampCalibrationHood(double value);
  double ClampCalibrationFlywheel(double value);
  // Matches WPILib rising-edge Debouncer: timer restarts on every false input.
  // Retain feed through normal speed sag after the initial debounce.
  class VolleyFeedGate
  {
  public:
    bool Calculate(bool held, double measuredRps, double targetRps, double nowSeconds);
    void Reset(double nowSeconds);

  private:
    bool feeding_ = false;
    double lastFalseTime_ = 0;
  };
  struct ShotIntakeSettings
  {
    double retractSpeedMetersPerSecond, holdPositionMeters;
    explicit ShotIntakeSettings(double speed = kDefaultIntakeRetractSpeed, double position = kDefaultIntakeHoldMetres);
  };
} // namespace season
