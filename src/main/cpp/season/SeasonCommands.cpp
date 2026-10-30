#include "season/Mechanisms.h"
#include <frc2/command/Commands.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <cmath>
#include <memory>
namespace season
{
namespace
{
double Now() { return frc::Timer::GetFPGATimestamp().value(); }
// Stop every transport stage together.
void StopFeed(Mechanisms m)
{
  m.conveyor.Stop();
  m.bottomIndexer.Stop();
  m.topIndexer.Stop();
}
void Feed(Mechanisms m)
{
  m.conveyor.SetVoltage(hardware::kConveyorFeedVolts);
  m.bottomIndexer.SetVoltage(hardware::kBottomFeedVolts);
  m.topIndexer.SetVoltage(hardware::kTopFeedVolts);
}
void StopAcquisition(Mechanisms m)
{
  StopFeed(m);
  m.intake.Stop();
  m.rack.Stop();
}
frc::Pose2d RobotPose(base::Drive &drive) { return drive.Pose(); }
} // namespace
// Run the intake and transport while the command owns them.
frc2::CommandPtr Acquire(Mechanisms m)
{
  return frc2::cmd::RunEnd(
      [m]
      {
        m.rack.SetPosition(hardware::kRackDeploy);
        m.intake.SetVoltage(-10);
        m.conveyor.SetVoltage(5);
        m.bottomIndexer.SetVoltage(5);
        m.topIndexer.SetVoltage(-5);
      },
      [m] { StopAcquisition(m); }, {&m.rack, &m.intake, &m.conveyor, &m.bottomIndexer, &m.topIndexer});
}
// Reverse the intake path and stop it on interruption.
frc2::CommandPtr Eject(Mechanisms m)
{
  return frc2::cmd::RunEnd(
      [m]
      {
        m.intake.SetVoltage(10);
        m.conveyor.SetVoltage(-5);
        m.bottomIndexer.SetVoltage(10);
        m.topIndexer.SetVoltage(-10);
      },
      [m]
      {
        m.intake.Stop();
        StopFeed(m);
      },
      {&m.intake, &m.conveyor, &m.bottomIndexer, &m.topIndexer});
}
// Hold the rack deployed while keeping the intake stopped.
frc2::CommandPtr Deploy(Mechanisms m)
{
  return frc2::cmd::RunEnd(
      [m]
      {
        m.rack.SetPosition(.30);
        m.intake.Stop();
      },
      [m]
      {
        m.rack.Stop();
        m.intake.Stop();
      },
      {&m.rack, &m.intake});
}
// Retract the rack while moving fuel away from the intake.
frc2::CommandPtr Stow(Mechanisms m)
{
  return frc2::cmd::RunEnd(
      [m]
      {
        m.rack.SetPosition(.12);
        m.intake.SetVoltage(-10);
        m.conveyor.SetVoltage(10);
      },
      [m]
      {
        m.rack.Stop();
        m.intake.Stop();
        m.conveyor.Stop();
      },
      {&m.rack, &m.intake, &m.conveyor});
}
// Clamp preset targets before latching them into the mechanisms.
frc2::CommandPtr ShooterPreset(Mechanisms m, double hood, double rps)
{
  return frc2::cmd::RunOnce(
      [m, hood, rps]
      {
        m.hood.SetPosition(ClampCalibrationHood(hood));
        m.shooter.SetVelocity(ClampCalibrationFlywheel(rps));
      },
      {&m.hood, &m.shooter});
}
// Sample calibration mode at command start.
frc2::CommandPtr DriverHubPreset(Mechanisms m)
{
  auto calibration = std::make_shared<bool>(false);
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("TunableNumbers/ShooterCalibration");
  return frc2::cmd::RunEnd(
             [m, table, calibration]
             {
               m.hood.SetPosition(*calibration ? ClampCalibrationHood(table->GetNumber("HoodRotations", 0))
                                               : hardware::kHubHood);
               m.shooter.SetVelocity(*calibration ? ClampCalibrationFlywheel(table->GetNumber("FlywheelRPS", 30))
                                                  : hardware::kHubRps);
             },
             [m]
             {
               m.hood.Stop();
               m.shooter.Stop();
               frc::SmartDashboard::PutBoolean("Shooter/CalibrationMode", false);
             },
             {&m.hood, &m.shooter})
      .BeforeStarting(
          [table, calibration]
          {
            *calibration = table->GetBoolean("Enabled", false);
            frc::SmartDashboard::PutBoolean("Shooter/CalibrationMode", *calibration);
          });
}
// Clear shooter speed and return the hood target to zero.
frc2::CommandPtr ShooterIdle(Mechanisms m)
{
  return frc2::cmd::RunOnce(
      [m]
      {
        m.hood.SetPosition(0);
        m.shooter.Stop();
      },
      {&m.hood, &m.shooter});
}
// Gate feeding on measured shooter speed and signal status to the driver.
frc2::CommandPtr AgitateAndFeed(Mechanisms m, std::function<void(double)> rumble)
{
  struct State
  {
    VolleyFeedGate gate;
    double start;
    bool feeding = false;
    ShotIntakeSettings settings;
  };
  auto state = std::make_shared<State>();
  frc::Preferences::InitDouble("Intake/ShotRetractSpeedMps", .10);
  frc::Preferences::InitDouble("Intake/ShotHoldPositionMeters", .13);
  return frc2::cmd::RunEnd(
             [m, rumble, state]
             {
               double now = Now();
               bool feeding = state->gate.Calculate(m.shooter.VelocityControlActive(), m.shooter.Velocity(),
                                                    m.shooter.VelocitySetpoint(), now);
               m.intake.SetVoltage(-10);
               if (feeding)
               {
                 Feed(m);
                 if (!state->feeding)
                   state->settings =
                       ShotIntakeSettings{frc::Preferences::GetDouble("Intake/ShotRetractSpeedMps", .10),
                                          frc::Preferences::GetDouble("Intake/ShotHoldPositionMeters", .13)};
                 m.rack.SetPositionDynamic(state->settings.holdPositionMeters,
                                           state->settings.retractSpeedMetersPerSecond, 100);
               }
               else
               {
                 StopFeed(m);
                 m.rack.SetPosition(.30);
               }
               state->feeding = feeding;
               double phase = std::fmod(now - state->start, 1.);
               rumble(feeding ? .35 : (phase < .15 || (phase >= .30 && phase < .45)) ? .75 : 0);
               frc::SmartDashboard::PutBoolean("Shooter/Feeding", feeding);
               frc::SmartDashboard::PutString("Shooter/FeedBlockedReason",
                                              feeding ? ""
                                              : !m.shooter.VelocityControlActive() || m.shooter.VelocitySetpoint() <= 0
                                                  ? "SELECT_SHOOTER_PRESET"
                                                  : "WAITING_FOR_SHOOTER_SPEED");
             },
             [m, rumble]
             {
               StopFeed(m);
               m.intake.Stop();
               if (frc::DriverStation::IsTeleopEnabled())
                 m.rack.SetPosition(.30);
               else
                 m.rack.Stop();
               rumble(0);
               frc::SmartDashboard::PutBoolean("Shooter/Feeding", false);
               frc::SmartDashboard::PutString("Shooter/FeedBlockedReason", "");
             },
             {&m.rack, &m.intake, &m.conveyor, &m.bottomIndexer, &m.topIndexer})
      .BeforeStarting(
          [state]
          {
            state->start = Now();
            state->gate.Reset(state->start);
            state->feeding = false;
          });
}
// Coordinate aiming, readiness, feeding, and interruption cleanup.
frc2::CommandPtr CoordinatedShot(Mechanisms m, base::Drive &drive, FieldGeometry field,
                                 std::function<ShotTarget()> supplier, CoordinatedShotOptions options)
{
  struct State
  {
    ShotTarget target{ShotMode::Hub, {}};
    double start = 0, lastFalse = 0, previous = 0, feedElapsed = 0;
    bool started = false, feeding = false;
    VolleyFeedGate volley;
  };
  auto state = std::make_shared<State>();
  std::vector<frc2::Subsystem *> requirements{&m.hood, &m.shooter, &m.bottomIndexer, &m.topIndexer, &m.conveyor};
  if (options.ownDrive)
    requirements.push_back(&drive);
  auto action =
      frc2::cmd::RunEnd(
          [m, &drive, field, options, state]
          {
            double now = Now();
            if (state->feeding)
              state->feedElapsed += now - state->previous;
            state->previous = now;
            auto pose = RobotPose(drive);
            bool changed = false;
            if (state->target.mode == ShotMode::Ferry)
            {
              auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
              auto selected = FerryTarget(alliance, pose, field);
              changed = selected.position.X() != state->target.position.X() ||
                        selected.position.Y() != state->target.position.Y();
              state->target = selected;
            }
            auto solution = CalculateShot(pose, state->target, field, options.simulation);
            bool calibration = options.calibration(), poseReady = options.poseReady(), force = options.forceFeed();
            if (calibration)
            {
              solution.hoodRotations = ClampCalibrationHood(options.calibrationHood());
              solution.flywheelRps = ClampCalibrationFlywheel(options.calibrationRps());
            }
            if (poseReady || force)
            {
              m.hood.SetPosition(solution.hoodRotations);
              m.shooter.SetVelocity(solution.flywheelRps);
            }
            else
            {
              m.hood.SetPosition(0);
              m.shooter.Stop();
            }
            auto speeds = drive.Speeds();
            auto raw =
                EvaluateReadiness(poseReady, solution, speeds.vx.value(), speeds.vy.value(), speeds.omega.value(),
                                  pose.Rotation().Radians().value(), m.hood.Position(), m.shooter.Velocity());
            bool readyCondition = !changed && !calibration && raw == Readiness::Ready;
            if (!readyCondition)
              state->lastFalse = now;
            bool ready = options.autonomous
                             ? readyCondition && now - state->lastFalse >= .15
                             : state->volley.Calculate(force, m.shooter.Velocity(), solution.flywheelRps, now);
            if (state->target.mode == ShotMode::Ferry && raw == Readiness::OutOfRange)
              raw = Readiness::UnsafeFerryPath;
            auto readiness = force         ? (ready ? Readiness::ForceFeedOverride : Readiness::FlywheelSpinningUp)
                             : calibration ? Readiness::CalibrationMode
                                           : raw;
            frc::SmartDashboard::PutString("Shooter/Readiness", ReadinessName(readiness));
            frc::SmartDashboard::PutString("Shooter/TargetMode", state->target.mode == ShotMode::Hub ? "HUB" : "FERRY");
            frc::SmartDashboard::PutNumber("Shooter/DistanceToTargetMeters", solution.distanceMeters);
            frc::SmartDashboard::PutBoolean("Shooter/SolutionCalibrated", solution.calibrated);
            frc::SmartDashboard::PutBoolean("Shooter/CalibrationMode", calibration);
            state->feeding = ready;
            if (ready)
            {
              Feed(m);
              state->started = true;
            }
            else
              StopFeed(m);
            options.readyFeedback(ready);
            double error = std::atan2(std::sin(solution.headingRadians - pose.Rotation().Radians().value()),
                                      std::cos(solution.headingRadians - pose.Rotation().Radians().value()));
            if (options.ownDrive)
              drive.RunVelocity(frc::ChassisSpeeds{
                  units::meters_per_second_t{0}, units::meters_per_second_t{0},
                  units::radians_per_second_t{(poseReady || force) ? std::clamp(error * 7., -7., 7.) : 0}});
            frc::SmartDashboard::PutBoolean("Shooter/Ready", ready);
            frc::SmartDashboard::PutBoolean("Shooter/Feeding", ready);
          },
          [m, &drive, options]
          {
            StopFeed(m);
            m.shooter.Stop();
            m.hood.Stop();
            if (options.ownDrive)
              drive.Stop();
            options.readyFeedback(false);
            frc::SmartDashboard::PutBoolean("Shooter/Ready", false);
            frc::SmartDashboard::PutBoolean("Shooter/Feeding", false);
          },
          frc2::Requirements{std::span<frc2::Subsystem *const>{requirements}})
          .BeforeStarting(
              [state, supplier]
              {
                state->target = supplier();
                state->start = state->lastFalse = state->previous = Now();
                state->feedElapsed = 0;
                state->started = state->feeding = false;
                state->volley.Reset(state->start);
              });
  if (options.autonomous)
    return std::move(action)
        .Until(
            [state, options]
            {
              return state->feedElapsed >= options.feedSeconds ||
                     (!state->started && Now() - state->start >= options.readyTimeoutSeconds);
            })
        .WithTimeout(units::second_t{options.totalTimeoutSeconds});
  return action;
}
} // namespace season
