#include "season/shooting/Shooting.h"
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
      return p.Translation() + frc::Translation2d{units::meter_t{kShooterExitXMetres}, units::meter_t{0}}.RotateBy(p.Rotation());
    }
    double Wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
    double Pitch(double hood) { return kPi / 2 - (kShooterRestDeflectionDegrees * kPi / 180 + hood * 2 * kPi); }
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
      double pitch = Pitch(hood) - kConservativePitchDegrees * kPi / 180, speed = rps * kProjectileSpeedPerRps * kConservativeSpeedScale;
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
    return {units::meter_t{(a == frc::DriverStation::Alliance::kBlue ? kBlueHubXInches : kRedHubXInches) * .0254},
            units::meter_t{kHubYInches * .0254}};
  }
  // The alliance zone is mirrored across the field length.
  double AllianceBoundary(frc::DriverStation::Alliance a, const FieldGeometry &f)
  {
    return a == frc::DriverStation::Alliance::kBlue ? kAllianceBoundaryInches * .0254 : f.length - kAllianceBoundaryInches * .0254;
  }
  bool InAllianceZone(frc::DriverStation::Alliance a, frc::Translation2d p, const FieldGeometry &f)
  {
    return a == frc::DriverStation::Alliance::kBlue ? p.X().value() <= AllianceBoundary(a, f)
                                                    : p.X().value() >= AllianceBoundary(a, f);
  }
  bool SeesAllianceSideTag(frc::DriverStation::Alliance a, const std::vector<int> &ids)
  {
    int first = a == frc::DriverStation::Alliance::kBlue ? kBlueFirstTag : kRedFirstTag,
        last = a == frc::DriverStation::Alliance::kBlue ? kBlueLastTag : kRedLastTag;
    return std::any_of(ids.begin(), ids.end(), [=](int id)
                       { return id >= first && id <= last; });
  }
  // Choose the near-side ferry destination for the current half of the field.
  ShotTarget FerryTarget(frc::DriverStation::Alliance a, frc::Pose2d robot, const FieldGeometry &f)
  {
    double direction = a == frc::DriverStation::Alliance::kBlue ? -1 : 1;
    frc::Translation2d reference{units::meter_t{AllianceBoundary(a, f) + direction * kFerryInsetMetres},
                                 units::meter_t{Reference(a, f).Y().value() + direction * kFerryTrenchOffsetMetres}};
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
    double neutral = center + direction * kHubHalfWidthMetres, wall = center - direction * kHubHalfWidthMetres;
    double radius = kHubAvoidanceRadiusMetres;
    bool safe = InAllianceZone(a, target.position, f) &&
                (a == frc::DriverStation::Alliance::kBlue ? start.X().value() > neutral : start.X().value() < neutral) &&
                SegmentDistance(HubPosition(a), start, target.position) > radius;
    if (safe)
      for (int step = 0; step <= kFerryHoodSearchSteps; ++step)
      {
        double hood = maxHood - (maxHood - kHoodMin) * step / static_cast<double>(kFerryHoodSearchSteps), rps = FerryFlywheelRps(distance, hood);
        double minHeight = kTrenchClearanceHeightMetres;
        if (std::isfinite(rps) && rps <= maxRps && HeightAtX(start, target.position, neutral, hood, rps) >= minHeight &&
            HeightAtX(start, target.position, wall, hood, rps) >= minHeight)
          return {hood, rps, true};
      }
    return {maxHood, std::clamp(FerryFlywheelRps(distance, maxHood), 0.0, maxRps), false};
  }
  // Interpolate the shot table using distance from the shooter exit.
  ShotSolution CalculateShot(frc::Pose2d robot, frc::Translation2d target, ShotCalibration calibration)
  {
    frc::Translation2d delta = (target - Exit(robot));
    double distance = delta.Norm().value();
    double hood = Interpolate(distance, kHoodTable);
    double rps = Interpolate(distance, kFlywheelTable);
    bool valid = distance >= kMinDistance && distance <= kMaxDistance;
    if (calibration.hubSpeedModel == HubSpeedModel::BallisticOpening)
    {
      // Aim the nominal drag-free trajectory at the hub opening.
      const double pitch = Pitch(hood);
      const double cosine = std::cos(pitch);
      const double height = kReleaseHeight + distance * std::tan(pitch) - kHubOpeningHeightMetres;
      const double divisor = 2 * cosine * cosine * height;
      if (std::isfinite(divisor) && divisor > 0)
        rps = std::sqrt(9.80665 * distance * distance / divisor) / kProjectileSpeedPerRps;
      else
        valid = false;
    }
    return {distance, Wrap(std::atan2(delta.Y().value(), delta.X().value()) - calibration.shooterHeadingOffsetRadians),
            hood, rps, valid && std::isfinite(rps)};
  }
  ShotSolution CalculateShot(frc::Pose2d robot, ShotTarget target, const FieldGeometry &field, ShotCalibration calibration,
                             ShotTuning tuning)
  {
    auto solution = CalculateShot(robot, target.position, calibration);
    if (target.mode == ShotMode::Ferry)
    {
      auto plan = CalculateFerry(robot, target, field, tuning);
      solution.hoodRotations = plan.hoodRotations;
      solution.flywheelRps = plan.flywheelRps;
      solution.calibrated = plan.safe;
    }
    return solution;
  }
  // Solve in the field frame: the projectile inherits both chassis translation
  // and rotational velocity at the shooter exit. Safety is checked on the world
  // trajectory, not on the virtual point used to compensate the shooter aim.
  ShotSolution CalculateMovingFerry(frc::Pose2d robot, ShotTarget target, const FieldGeometry &f,
                                    frc::ChassisSpeeds robotSpeeds, ShotCalibration calibration, ShotTuning tuning)
  {
    auto invalid = CalculateShot(robot, target, f, calibration, tuning);
    invalid.calibrated = false;
    const auto field = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotSpeeds, robot.Rotation());
    const double vx = field.vx.value(), vy = field.vy.value(), omega = field.omega.value();
    if (target.mode != ShotMode::Ferry || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
        std::hypot(vx, vy) > kMovingFerryMaxTranslation || std::abs(omega) > kMovingFerryMaxOmega)
      return invalid;
    const auto start = Exit(robot);
    const auto offset = start - robot.Translation();
    const double launchVx = vx - omega * offset.Y().value(), launchVy = vy + omega * offset.X().value();
    const auto alliance = target.position.X().value() < f.length / 2 ? frc::DriverStation::Alliance::kBlue
                                                                     : frc::DriverStation::Alliance::kRed;
    const double direction = alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
    const double center = Reference(alliance, f).X().value();
    const double neutral = center + direction * kHubHalfWidthMetres, wall = center - direction * kHubHalfWidthMetres;
    const double radius = kHubAvoidanceRadiusMetres;
    if (!InAllianceZone(alliance, target.position, f) || direction * (start.X().value() - neutral) <= 0 ||
        SegmentDistance(HubPosition(alliance), start, target.position) <= radius)
      return invalid;
    const double dx = target.position.X().value() - start.X().value();
    const double dy = target.position.Y().value() - start.Y().value();
    const double maxHood = std::clamp(tuning.ferryHoodRotations, kHoodMin, kHoodMax);
    const double maxRps = std::max(0., tuning.ferryFlywheelRps);
    for (int step = 0; step <= kFerryHoodSearchSteps; ++step)
    {
      const double hood = maxHood - (maxHood - kHoodMin) * step / static_cast<double>(kFerryHoodSearchSteps);
      const double pitch = Pitch(hood);
      auto residual = [&](double t)
      {
        return std::hypot(dx / t - launchVx, dy / t - launchVy) * std::tan(pitch) -
               (.5 * 9.80665 * t - kReleaseHeight / t);
      };
      double lo = kFlightMinSeconds, hi = kFlightMaxSeconds;
      if (!(residual(lo) > 0 && residual(hi) < 0))
        continue;
      for (int i = 0; i < kFlightSearchIterations; ++i)
      {
        const double mid = (lo + hi) / 2;
        if (residual(mid) > 0)
          lo = mid;
        else
          hi = mid;
      }
      const double t = (lo + hi) / 2;
      const double ux = dx / t - launchVx, uy = dy / t - launchVy;
      const double horizontal = std::hypot(ux, uy);
      const double rps = horizontal / std::cos(pitch) / kProjectileSpeedPerRps;
      if (!std::isfinite(rps) || rps > maxRps || horizontal < 1e-6)
        continue;
      // Repeat the clearance check with a one-degree lower and 1% slower launch,
      // matching the stationary solver's margin.
      const double conservativePitch = pitch - kConservativePitchDegrees * kPi / 180;
      const double speed = rps * kProjectileSpeedPerRps * kConservativeSpeedScale;
      const double horizontalSpeed = speed * std::cos(conservativePitch);
      const double wx = launchVx + horizontalSpeed * ux / horizontal;
      const double wy = launchVy + horizontalSpeed * uy / horizontal;
      const double wz = speed * std::sin(conservativePitch);
      const double landingTime = (wz + std::sqrt(wz * wz + 2 * 9.80665 * kReleaseHeight)) / 9.80665;
      const frc::Translation2d landing{units::meter_t{start.X().value() + wx * landingTime},
                                       units::meter_t{start.Y().value() + wy * landingTime}};
      if (!InAllianceZone(alliance, landing, f) || landing.Y().value() < 0 || landing.Y().value() > f.width ||
          SegmentDistance(HubPosition(alliance), start, landing) <= radius)
        continue;
      auto height = [&](double x)
      {
        if (std::abs(wx) < 1e-9)
          return -inf;
        const double crossing = (x - start.X().value()) / wx;
        if (crossing < 0 || crossing > t)
          return -inf;
        return kReleaseHeight + wz * crossing - .5 * 9.80665 * crossing * crossing;
      };
      const double minHeight = kTrenchClearanceHeightMetres;
      if (height(neutral) < minHeight || height(wall) < minHeight)
        continue;
      return {start.Distance(target.position).value(), Wrap(std::atan2(uy, ux) - calibration.shooterHeadingOffsetRadians),
              hood, rps, true};
    }
    return invalid;
  }
  ShotSolution CalculateDriverAim(frc::DriverStation::Alliance a, frc::Pose2d robot, const FieldGeometry &field,
                                  ShotCalibration calibration)
  {
    auto target = DriverAimTarget(a, robot, field);
    if (target.mode == ShotMode::Hub)
      return CalculateShot(robot, target, field, calibration);

    const double direction = a == frc::DriverStation::Alliance::kBlue ? -1.0 : 1.0;
    const double shotHeading = direction < 0 ? kPi : 0.0;
    const double robotHeading = Wrap(shotHeading - calibration.shooterHeadingOffsetRadians);
    const frc::Pose2d aligned{robot.Translation(), frc::Rotation2d{units::radian_t{robotHeading}}};
    target.position = {units::meter_t{AllianceBoundary(a, field) + direction * kFerryInsetMetres}, robot.Y()};
    auto solution = CalculateShot(aligned, target, field, calibration);
    solution.headingRadians = robotHeading;
    return solution;
  }
  const char *ReadinessName(Readiness value)
  {
    static constexpr const char *names[]{
        "POSE_NOT_READY", "OUT_OF_RANGE", "UNSAFE_FERRY_PATH", "MOVING_TOO_FAST", "AIMING",
        "HOOD_MOVING", "FLYWHEEL_SPINNING_UP", "CALIBRATION_MODE", "READY", "FORCE_FEED_OVERRIDE"};
    return names[static_cast<unsigned>(value)];
  }
  // Check feedback and alignment before allowing feed.
  Readiness EvaluateReadiness(bool poseReady, ShotSolution s, double vx, double vy, double omega, double heading,
                              double hood, double flywheel, double maxTranslation, double maxOmega)
  {
    if (!poseReady || !std::isfinite(heading))
      return Readiness::PoseNotReady;
    if (!s.calibrated || !std::isfinite(s.distanceMeters) || !std::isfinite(s.headingRadians) ||
        !std::isfinite(s.hoodRotations) || !std::isfinite(s.flywheelRps))
      return Readiness::OutOfRange;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) || std::hypot(vx, vy) > maxTranslation ||
        std::abs(omega) > maxOmega)
      return Readiness::MovingTooFast;
    if (std::abs(Wrap(heading - s.headingRadians)) > kHeadingToleranceDegrees * kPi / 180)
      return Readiness::Aiming;
    if (!std::isfinite(hood) || std::abs(hood - s.hoodRotations) > kHoodToleranceRotations)
      return Readiness::HoodMoving;
    if (!std::isfinite(flywheel) || std::abs(flywheel - s.flywheelRps) > kFlywheelToleranceRps)
      return Readiness::FlywheelSpinningUp;
    return Readiness::Ready;
  }
  double ClampCalibrationHood(double value) { return std::isfinite(value) ? std::clamp(value, kHoodMin, kHoodMax) : 0; }
  double ClampCalibrationFlywheel(double value) { return std::isfinite(value) ? std::clamp(value, 0.0, kMaxCalibrationFlywheelRps) : 0; }
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
      bool ready = std::abs(measured - target) <= kFeedReadinessToleranceRps;
      if (!ready)
        lastFalseTime_ = now;
      feeding_ = ready && now - lastFalseTime_ >= kDebounceSeconds;
    }
    return feeding_;
  }
  ShotIntakeSettings::ShotIntakeSettings(double speed, double position)
      : retractSpeedMetersPerSecond(std::isfinite(speed) ? std::clamp(speed, kIntakeMinRetractSpeed, kIntakeMaxRetractSpeed) : kDefaultIntakeRetractSpeed),
        holdPositionMeters(std::isfinite(position) ? std::clamp(position, kIntakeMinHoldMetres, kIntakeMaxHoldMetres) : kDefaultIntakeHoldMetres)
  {
  }
} // namespace season
