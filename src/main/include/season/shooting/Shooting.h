#pragma once
#include <array>
#include <cmath>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace season
{
  inline constexpr double kPi = 3.14159265358979323846;
  inline constexpr double kMovingFerryMaxTranslation = 1., kMovingFerryMaxOmega = .3;
  inline constexpr double kFlightMinSeconds = .05, kFlightMaxSeconds = 5.;
  inline constexpr int kFlightSearchIterations = 45;
  inline constexpr double kHeadingToleranceDegrees = 2, kHoodToleranceRotations = .002, kFlywheelToleranceRps = .75;
  inline constexpr double kMaxCalibrationFlywheelRps = 60., kFeedReadinessToleranceRps = 3;
  inline constexpr double kIntakeMinRetractSpeed = .01, kIntakeMaxRetractSpeed = .50;
  inline constexpr double kIntakeMinHoldMetres = .13, kIntakeMaxHoldMetres = .30;
  inline constexpr double kDefaultShotMaxTranslation = .25, kDefaultShotMaxOmega = 10 * kPi / 180;
  inline constexpr double kDefaultIntakeRetractSpeed = .10, kDefaultIntakeHoldMetres = .13;
  inline constexpr double kDefaultFerryHoodRotations = .049, kDefaultFerryFlywheelRps = 55;
  inline constexpr double kShooterExitXMetres = -.19, kShooterRestDeflectionDegrees = 16;
  inline constexpr double kBlueHubXInches = 182.105, kRedHubXInches = 469.115, kHubYInches = 158.845;
  inline constexpr double kAllianceBoundaryInches = 158.6, kFerryInsetMetres = .90, kFerryTrenchOffsetMetres = .40;
  inline constexpr int kBlueFirstTag = 17, kBlueLastTag = 32, kRedFirstTag = 1, kRedLastTag = 16;
  inline constexpr double kHubWidthInches = 47, kFuelDiameterInches = 5.91;
  inline constexpr double kHubClearanceMetres = .12, kShotSafetyMarginMetres = .10;
  inline constexpr double kTrenchHeightInches = 40.25, kHubOpeningHeightInches = 72;
  inline constexpr double kHubHalfWidthMetres = kHubWidthInches * .0254 / 2;
  inline constexpr double kFuelRadiusMetres = kFuelDiameterInches * .0254 / 2;
  inline const double kHubAvoidanceRadiusMetres = std::hypot(kHubHalfWidthMetres, kHubHalfWidthMetres) +
      kFuelRadiusMetres + kHubClearanceMetres + kShotSafetyMarginMetres;
  inline constexpr double kTrenchClearanceHeightMetres = kTrenchHeightInches * .0254 +
      kFuelRadiusMetres + kShotSafetyMarginMetres;
  inline constexpr double kHubOpeningHeightMetres = kHubOpeningHeightInches * .0254;
  inline constexpr int kFerryHoodSearchSteps = 50;
  inline constexpr double kConservativePitchDegrees = 1, kConservativeSpeedScale = .99;
  inline constexpr double kHoodMin = 0, kHoodMax = .049;
  inline constexpr double kMinDistance = 46.003 * .0254, kMaxDistance = 236 * .0254;
  inline constexpr double kReleaseHeight = .45;
  inline constexpr double kMainWheelSpeedPerRps = 4 * kPi * .0254;
  // Counter-wheel surface speed / main-wheel surface speed (gearing and diameters).
  inline constexpr double kCounterWheelSurfaceSpeedRatio = (48.0 / 54) * (1.5 / 4);
  // Unequal contact speeds create backspin and reduce translational exit speed.
  // This is a speed ratio, not an energy-efficiency factor; do not apply it twice.
  inline constexpr double kBackspinTranslationSpeedScale = (1 + kCounterWheelSurfaceSpeedRatio) / 2;
  inline constexpr double kShooterSurfaceEfficiencyScale = 28.5 / 30;
  inline constexpr double kProjectileSpeedPerRps =
      kMainWheelSpeedPerRps * kBackspinTranslationSpeedScale * kShooterSurfaceEfficiencyScale;
  inline constexpr double kDebounceSeconds = .15;
  inline constexpr std::array<double, 8> kDistanceInches{46.003, 123.24, 144.60, 153.64, 191.86, 202.78, 210.47, 236};
  inline constexpr std::array<double, 8> kHoodTable{0, .019, .020, .020, .020, .020, .020, .049};
  // Table entries are wheel-RPS targets, not ball speeds; do not scale them again.
  inline constexpr std::array<double, 8> kFlywheelTable{30, 37.9, 40, 41, 46, 47, 47, 53.7};
}

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
