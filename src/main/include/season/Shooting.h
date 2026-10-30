#pragma once
#include <array>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

// Pure season algorithms. Distances are meters, angles radians, mechanism
// positions rotations, velocities rotations/second; time is monotonic seconds.
namespace season
{
inline constexpr double kPi = 3.14159265358979323846;
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
  double ferryHoodRotations = .049, ferryFlywheelRps = 55;
};
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
                            double hood, double flywheel);
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
  explicit ShotIntakeSettings(double speed = .10, double position = .13);
};
} // namespace season
