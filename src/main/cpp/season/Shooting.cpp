#include "season/Shooting.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace season
{
namespace
{
constexpr double inf = std::numeric_limits<double>::infinity();
frc::Translation2d Exit(frc::Pose2d p)
{
  return p.Translation() + frc::Translation2d{units::meter_t{-.19}, units::meter_t{0}}.RotateBy(p.Rotation());
}
double Wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double Pitch(double hood) { return kPi / 2 - (16 * kPi / 180 + hood * 2 * kPi); }
double Interpolate(double distance, const std::array<double, 8> &values)
{
  if (distance <= kDistanceInches.front() * .0254)
    return values.front();
  for (std::size_t i = 1; i < values.size(); ++i)
  {
    if (distance <= kDistanceInches[i] * .0254)
    {
      double a = kDistanceInches[i - 1] * .0254, b = kDistanceInches[i] * .0254;
      return values[i - 1] + (values[i] - values[i - 1]) * (distance - a) / (b - a);
    }
  }
  return values.back();
}
frc::Translation2d Reference(frc::DriverStation::Alliance a, const FieldGeometry &f)
{
  return a == frc::DriverStation::Alliance::kBlue ? f.blueTrenchReference : f.redTrenchReference;
}
double SegmentDistance(frc::Translation2d point, frc::Translation2d start, frc::Translation2d end)
{
  frc::Translation2d segment = (end - start);
  double squared = segment.X().value() * segment.X().value() + segment.Y().value() * segment.Y().value();
  if (squared <= 1e-9)
    return point.Distance(start).value();
  frc::Translation2d v = (point - start);
  double t =
      std::clamp((v.X().value() * segment.X().value() + v.Y().value() * segment.Y().value()) / squared, 0.0, 1.0);
  return point.Distance(start + segment * t).value();
}
double HeightAtX(frc::Translation2d start, frc::Translation2d target, double x, double hood, double rps)
{
  double dx = target.X().value() - start.X().value();
  if (std::abs(dx) < 1e-9)
    return -inf;
  double fraction = (x - start.X().value()) / dx;
  if (fraction < 0 || fraction > 1)
    return -inf;
  double pitch = Pitch(hood) - kPi / 180, speed = rps * kProjectileSpeedPerRps * .99;
  double horizontal = speed * std::cos(pitch);
  if (horizontal <= 0)
    return -inf;
  double time = target.Distance(start).value() * fraction / horizontal;
  return kReleaseHeight + speed * std::sin(pitch) * time - .5 * 9.80665 * time * time;
}
} // namespace
// Field coordinates use the shared blue-origin frame.
frc::Translation2d HubPosition(frc::DriverStation::Alliance a)
{
  return {units::meter_t{(a == frc::DriverStation::Alliance::kBlue ? 182.105 : 469.115) * .0254},
          units::meter_t{158.845 * .0254}};
}
// The alliance zone is mirrored across the field length.
double AllianceBoundary(frc::DriverStation::Alliance a, const FieldGeometry &f)
{
  return a == frc::DriverStation::Alliance::kBlue ? 158.6 * .0254 : f.length - 158.6 * .0254;
}
bool InAllianceZone(frc::DriverStation::Alliance a, frc::Translation2d p, const FieldGeometry &f)
{
  return a == frc::DriverStation::Alliance::kBlue ? p.X().value() <= AllianceBoundary(a, f)
                                                  : p.X().value() >= AllianceBoundary(a, f);
}
bool SeesAllianceSideTag(frc::DriverStation::Alliance a, const std::vector<int> &ids)
{
  int first = a == frc::DriverStation::Alliance::kBlue ? 17 : 1,
      last = a == frc::DriverStation::Alliance::kBlue ? 32 : 16;
  return std::any_of(ids.begin(), ids.end(), [=](int id) { return id >= first && id <= last; });
}
// Choose the near-side ferry destination for the current half of the field.
ShotTarget FerryTarget(frc::DriverStation::Alliance a, frc::Pose2d robot, const FieldGeometry &f)
{
  double direction = a == frc::DriverStation::Alliance::kBlue ? -1 : 1;
  frc::Translation2d reference{units::meter_t{AllianceBoundary(a, f) + direction * .90},
                               units::meter_t{Reference(a, f).Y().value() + direction * .40}};
  double low = std::min(reference.Y().value(), f.width - reference.Y().value()),
         high = std::max(reference.Y().value(), f.width - reference.Y().value());
  return {ShotMode::Ferry, {reference.X(), units::meter_t{robot.Y().value() <= f.width / 2 ? low : high}}};
}
ShotTarget DriverAimTarget(frc::DriverStation::Alliance a, frc::Pose2d robot, const FieldGeometry &f)
{
  return InAllianceZone(a, robot.Translation(), f) ? ShotTarget{ShotMode::Hub, HubPosition(a)}
                                                   : FerryTarget(a, robot, f);
}
// Autonomous hub targeting applies inside the alliance zone or with an alliance tag.
ShotTarget SelectTarget(frc::DriverStation::Alliance a, frc::Pose2d robot, const std::vector<int> &ids,
                        const FieldGeometry &f)
{
  return InAllianceZone(a, robot.Translation(), f) || SeesAllianceSideTag(a, ids)
             ? ShotTarget{ShotMode::Hub, HubPosition(a)}
             : FerryTarget(a, robot, f);
}
// Estimate launch speed from the projectile trajectory.
double FerryFlywheelRps(double distance, double hood)
{
  double pitch = Pitch(hood), cosine = std::cos(pitch), height = kReleaseHeight + distance * std::tan(pitch);
  if (!std::isfinite(distance) || distance < 0 || cosine <= 0 || height <= 0)
    return inf;
  return std::sqrt(9.80665 * distance * distance / (2 * cosine * cosine * height)) / kProjectileSpeedPerRps;
}
// Reject trajectories that do not clear the field structures.
FerryPlan CalculateFerry(frc::Pose2d robot, ShotTarget target, const FieldGeometry &f, ShotTuning tuning)
{
  frc::Translation2d start = Exit(robot);
  double distance = start.Distance(target.position).value();
  double maxHood = std::clamp(tuning.ferryHoodRotations, kHoodMin, kHoodMax),
         maxRps = std::max(0.0, tuning.ferryFlywheelRps);
  frc::DriverStation::Alliance a = target.position.X().value() < f.length / 2 ? frc::DriverStation::Alliance::kBlue
                                                                              : frc::DriverStation::Alliance::kRed;
  double direction = a == frc::DriverStation::Alliance::kBlue ? 1 : -1, center = Reference(a, f).X().value();
  double neutral = center + direction * 47 * .0254 / 2, wall = center - direction * 47 * .0254 / 2;
  double radius = std::hypot(47 * .0254 / 2, 47 * .0254 / 2) + 5.91 * .0254 / 2 + .12 + .10;
  bool safe = InAllianceZone(a, target.position, f) &&
              (a == frc::DriverStation::Alliance::kBlue ? start.X().value() > neutral : start.X().value() < neutral) &&
              SegmentDistance(HubPosition(a), start, target.position) > radius;
  if (safe)
    for (int step = 0; step <= 50; ++step)
    {
      double hood = maxHood - (maxHood - kHoodMin) * step / 50, rps = FerryFlywheelRps(distance, hood);
      double minHeight = 40.25 * .0254 + 5.91 * .0254 / 2 + .10;
      if (std::isfinite(rps) && rps <= maxRps && HeightAtX(start, target.position, neutral, hood, rps) >= minHeight &&
          HeightAtX(start, target.position, wall, hood, rps) >= minHeight)
        return {hood, rps, true};
    }
  return {maxHood, std::clamp(FerryFlywheelRps(distance, maxHood), 0.0, maxRps), false};
}
// Interpolate the shot table using distance from the shooter exit.
ShotSolution CalculateShot(frc::Pose2d robot, frc::Translation2d target, bool simulation)
{
  frc::Translation2d delta = (target - Exit(robot));
  double distance = delta.Norm().value();
  return {distance, Wrap(std::atan2(delta.Y().value(), delta.X().value()) - (simulation ? 0 : kPi)),
          Interpolate(distance, kHoodTable), Interpolate(distance, kFlywheelTable),
          distance >= kMinDistance && distance <= kMaxDistance};
}
ShotSolution CalculateShot(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field, bool simulation,
                           ShotTuning tuning)
{
  auto solution = CalculateShot(robot, target.position, simulation);
  if (target.mode == ShotMode::Ferry)
  {
    auto plan = CalculateFerry(robot, target, field, tuning);
    solution.hoodRotations = plan.hoodRotations;
    solution.flywheelRps = plan.flywheelRps;
    solution.calibrated = plan.safe;
  }
  return solution;
}
ShotSolution CalculateDriverAim(frc::DriverStation::Alliance a, frc::Pose2d robot, const FieldGeometry &field,
                                bool simulation)
{
  auto target = DriverAimTarget(a, robot, field);
  if (target.mode == ShotMode::Hub)
    return CalculateShot(robot, target, field, simulation);

  const double direction = a == frc::DriverStation::Alliance::kBlue ? -1.0 : 1.0;
  const double shotHeading = direction < 0 ? kPi : 0.0;
  const double robotHeading = Wrap(shotHeading - (simulation ? 0.0 : kPi));
  const frc::Pose2d aligned{robot.Translation(), frc::Rotation2d{units::radian_t{robotHeading}}};
  target.position = {units::meter_t{AllianceBoundary(a, field) + direction * .90}, robot.Y()};
  auto solution = CalculateShot(aligned, target, field, simulation);
  solution.headingRadians = robotHeading;
  return solution;
}
const char *ReadinessName(Readiness value)
{
  static constexpr const char *names[]{
      "POSE_NOT_READY", "OUT_OF_RANGE",         "UNSAFE_FERRY_PATH", "MOVING_TOO_FAST", "AIMING",
      "HOOD_MOVING",    "FLYWHEEL_SPINNING_UP", "CALIBRATION_MODE",  "READY",           "FORCE_FEED_OVERRIDE"};
  return names[static_cast<unsigned>(value)];
}
// Check feedback and alignment before allowing feed.
Readiness EvaluateReadiness(bool poseReady, ShotSolution s, double vx, double vy, double omega, double heading,
                            double hood, double flywheel)
{
  if (!poseReady || !std::isfinite(heading))
    return Readiness::PoseNotReady;
  if (!s.calibrated || !std::isfinite(s.distanceMeters) || !std::isfinite(s.headingRadians) ||
      !std::isfinite(s.hoodRotations) || !std::isfinite(s.flywheelRps))
    return Readiness::OutOfRange;
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) || std::hypot(vx, vy) > .25 ||
      std::abs(omega) > 10 * kPi / 180)
    return Readiness::MovingTooFast;
  if (std::abs(Wrap(heading - s.headingRadians)) > 2 * kPi / 180)
    return Readiness::Aiming;
  if (!std::isfinite(hood) || std::abs(hood - s.hoodRotations) > .002)
    return Readiness::HoodMoving;
  if (!std::isfinite(flywheel) || std::abs(flywheel - s.flywheelRps) > .75)
    return Readiness::FlywheelSpinningUp;
  return Readiness::Ready;
}
double ClampCalibrationHood(double value) { return std::isfinite(value) ? std::clamp(value, kHoodMin, kHoodMax) : 0; }
double ClampCalibrationFlywheel(double value) { return std::isfinite(value) ? std::clamp(value, 0.0, 60.0) : 0; }
void VolleyFeedGate::Reset(double now)
{
  feeding_ = false;
  lastFalseTime_ = now;
}
// Debounce spin-up before feeding; invalid feedback closes the gate.
bool VolleyFeedGate::Calculate(bool held, double measured, double target, double now)
{
  // A volley may tolerate real speed sag after spin-up, never invalid feedback.
  if (!held || !std::isfinite(measured) || !std::isfinite(target) || target <= 0 || !std::isfinite(now))
  {
    Reset(now);
    return false;
  }
  if (!feeding_)
  {
    bool ready = std::abs(measured - target) <= 3;
    if (!ready)
      lastFalseTime_ = now;
    feeding_ = ready && now - lastFalseTime_ >= kDebounceSeconds;
  }
  return feeding_;
}
ShotIntakeSettings::ShotIntakeSettings(double speed, double position)
    : retractSpeedMetersPerSecond(std::isfinite(speed) ? std::clamp(speed, .01, .50) : .10),
      holdPositionMeters(std::isfinite(position) ? std::clamp(position, .13, .30) : .13)
{
}
} // namespace season
