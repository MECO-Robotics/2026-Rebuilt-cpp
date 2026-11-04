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
  // Pi used for season geometry and angular conversions.
  inline constexpr double kPi = 3.14159265358979323846;
  // Maximum chassis translation and angular speeds allowed for a moving ferry shot.
  inline constexpr double kMovingFerryMaxTranslation = 1., kMovingFerryMaxOmega = .3;
  // Lower and upper bounds for the projectile flight-time search.
  inline constexpr double kFlightMinSeconds = .05, kFlightMaxSeconds = 5.;
  // Iterations used by the bounded flight-time search.
  inline constexpr int kFlightSearchIterations = 45;
  // Readiness evaluator tolerances for heading, hood position, and flywheel speed.
  // Autonomous waits for this readiness before beginning a volley.
  inline constexpr double kHeadingToleranceDegrees = 2, kHoodToleranceRotations = .002, kFlywheelToleranceRps = .75;
  // Highest flywheel speed accepted by the calibration workflow.
  inline constexpr double kMaxCalibrationFlywheelRps = 60.;
  // Flywheel error in RPS accepted to start the teleop feed gate; the gate then tolerates normal speed sag.
  inline constexpr double kFeedReadinessToleranceRps = 3;
  // Minimum and maximum intake speeds while retracting.
  inline constexpr double kIntakeMinRetractSpeed = .01, kIntakeMaxRetractSpeed = .50;
  // Minimum and maximum intake extensions while holding a game piece.
  inline constexpr double kIntakeMinHoldMetres = .13, kIntakeMaxHoldMetres = .30;
  // Default maximum chassis translation and angular speeds allowed during a shot.
  inline constexpr double kDefaultShotMaxTranslation = .25, kDefaultShotMaxOmega = 10 * kPi / 180;
  // Default intake retraction speed and hold extension during shot preparation.
  inline constexpr double kDefaultIntakeRetractSpeed = .10, kDefaultIntakeHoldMetres = .13;
  // Default hood position and flywheel speed for a ferry shot.
  inline constexpr double kDefaultFerryHoodRotations = .049, kDefaultFerryFlywheelRps = 55;
  // Shooter exit location relative to the robot center along robot X.
  inline constexpr double kShooterExitXMetres = -.19;
  // Nominal projectile deflection from the shooter at rest.
  inline constexpr double kShooterRestDeflectionDegrees = 16;
  // Blue and red hub center X coordinates and the shared hub center Y coordinate, in field inches.
  inline constexpr double kBlueHubXInches = 182.105, kRedHubXInches = 469.115, kHubYInches = 158.845;
  // X coordinate separating the alliance zones in field inches.
  inline constexpr double kAllianceBoundaryInches = 158.6;
  // Ferry target inset into the alliance zone from the alliance line.
  inline constexpr double kFerryInsetMetres = .90;
  // Lateral offset used to place ferry targets clear of the trench.
  inline constexpr double kFerryTrenchOffsetMetres = .40;
  // First and last AprilTag IDs assigned to the blue alliance side.
  inline constexpr int kBlueFirstTag = 17, kBlueLastTag = 32;
  // First and last AprilTag IDs assigned to the red alliance side.
  inline constexpr int kRedFirstTag = 1, kRedLastTag = 16;
  // Hub width and fuel diameter used for shot-clearance geometry, in inches.
  inline constexpr double kHubWidthInches = 47, kFuelDiameterInches = 5.91;
  // Hub clearance and extra safety margin applied to shot trajectories, in metres.
  inline constexpr double kHubClearanceMetres = .12, kShotSafetyMarginMetres = .10;
  // Trench height used to reject trajectories that collide with the trench.
  inline constexpr double kTrenchHeightInches = 40.25;
  // Hub opening height used by desktop ballistic aiming and the simulated fuel-scoring plane.
  inline constexpr double kHubOpeningHeightInches = 72;
  // Half the hub width converted to meters for clearance calculations.
  inline constexpr double kHubHalfWidthMetres = kHubWidthInches * .0254 / 2;
  // Fuel radius converted to meters for trajectory clearance calculations.
  inline constexpr double kFuelRadiusMetres = kFuelDiameterInches * .0254 / 2;
  // Radius around the hub that a candidate shot trajectory must clear.
  inline const double kHubAvoidanceRadiusMetres = std::hypot(kHubHalfWidthMetres, kHubHalfWidthMetres) +
      kFuelRadiusMetres + kHubClearanceMetres + kShotSafetyMarginMetres;
  // Minimum projectile-center height needed to clear the trench and margin.
  inline constexpr double kTrenchClearanceHeightMetres = kTrenchHeightInches * .0254 +
      kFuelRadiusMetres + kShotSafetyMarginMetres;
  // Hub opening height converted to meters for trajectory checks.
  inline constexpr double kHubOpeningHeightMetres = kHubOpeningHeightInches * .0254;
  // Candidate hood positions evaluated when calculating a ferry shot.
  inline constexpr int kFerryHoodSearchSteps = 50;
  // Pitch margin and launch-speed scale used for conservative trajectory clearance checks.
  inline constexpr double kConservativePitchDegrees = 1, kConservativeSpeedScale = .99;
  // Minimum and maximum hood output rotations accepted by the shot solver.
  inline constexpr double kHoodMin = 0, kHoodMax = .049;
  // Projectile release height above the floor in meters.
  inline constexpr double kReleaseHeight = .45;
  // Main-wheel surface speed in meters per second per wheel rotation/second.
  inline constexpr double kMainWheelSpeedPerRps = 4 * kPi * .0254;
  // Counter-wheel surface speed / main-wheel surface speed (gearing and diameters).
  inline constexpr double kCounterWheelSurfaceSpeedRatio = (48.0 / 54) * (1.5 / 4);
  // Unequal contact speeds create backspin and reduce translational exit speed.
  // This is a speed ratio, not an energy-efficiency factor; do not apply it twice.
  inline constexpr double kBackspinTranslationSpeedScale = (1 + kCounterWheelSurfaceSpeedRatio) / 2;
  // Empirical scale correcting ideal wheel surface speed to measured output.
  inline constexpr double kShooterSurfaceEfficiencyScale = 28.5 / 30;
  // Conversion from flywheel RPS to estimated projectile launch speed.
  inline constexpr double kProjectileSpeedPerRps =
      kMainWheelSpeedPerRps * kBackspinTranslationSpeedScale * kShooterSurfaceEfficiencyScale;
  // Readiness must stay true for this long before feeding starts.
  inline constexpr double kDebounceSeconds = .15;
  // Calibration distances paired by index with hood and flywheel tables.
  inline constexpr std::array<double, 8> kDistanceInches{46.003, 123.24, 144.60, 153.64, 191.86, 202.78, 210.47, 236};
  // Shot validity uses the first and last calibrated distances, converted to metres.
  inline constexpr double kMinDistance = kDistanceInches.front() * .0254;
  inline constexpr double kMaxDistance = kDistanceInches.back() * .0254;
  // Hood output-rotation targets paired with calibration distances.
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
  // Select the hub speed model and shooter direction independently of runtime.
  enum class HubSpeedModel { MeasuredTable, BallisticOpening };
  struct ShotCalibration
  {
    HubSpeedModel hubSpeedModel = HubSpeedModel::MeasuredTable;
    double shooterHeadingOffsetRadians = kPi;
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
  ShotSolution CalculateShot(frc::Pose2d robot, frc::Translation2d target, ShotCalibration calibration = {});
  ShotSolution CalculateShot(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field, ShotCalibration calibration = {},
                             ShotTuning tuning = {});
  // Speeds are measured robot-relative. Returns an invalid solution when no
  // bounded-velocity trajectory clears the field and lands in the alliance zone.
  ShotSolution CalculateMovingFerry(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field,
                                    frc::ChassisSpeeds robotSpeeds, ShotCalibration calibration = {}, ShotTuning tuning = {});
  ShotSolution CalculateDriverAim(frc::DriverStation::Alliance alliance, frc::Pose2d robot, const FieldGeometry &field,
                                  ShotCalibration calibration = {});
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
