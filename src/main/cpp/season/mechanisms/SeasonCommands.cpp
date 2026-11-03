#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/MechanismConfig.h"
#include "season/simulation/FuelConfig.h"
#include <frc2/command/Commands.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
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
  double ConfiguredFeedDurationSeconds()
  {
    double capacity = frc::Preferences::GetDouble("AutoDrive/EstimatedCapacity", 0);
    double rate = frc::Preferences::GetDouble("AutoDrive/FeedBallsPerSecond", 0);
    if (frc::RobotBase::IsSimulation())
    {
      if (capacity == 0)
        capacity = fuelconfig::kSimulationCapacity;
      if (rate == 0)
        rate = fuelconfig::kSimulationFeedBallsPerSecond;
    }
    if (!std::isfinite(capacity) || capacity <= 0 || !std::isfinite(rate) || rate <= 0)
      return 0;
    const double result = capacity / rate;
    return std::isfinite(result) && result > 0 ? result : 0;
  }
  // Run the intake and transport while the command owns them.
  frc2::CommandPtr Acquire(Mechanisms m)
  {
    return frc2::cmd::RunEnd(
        [m]
        {
          m.rack.SetPosition(hardware::kRackDeploy);
          m.intake.SetVoltage(hardware::kIntakeVolts);
          m.conveyor.SetVoltage(hardware::kAcquireConveyorVolts);
          m.bottomIndexer.SetVoltage(hardware::kAcquireBottomVolts);
          m.topIndexer.SetVoltage(hardware::kAcquireTopVolts);
        },
        [m]
        { StopAcquisition(m); }, {&m.rack, &m.intake, &m.conveyor, &m.bottomIndexer, &m.topIndexer});
  }
  // Reverse the intake path and stop it on interruption.
  frc2::CommandPtr Eject(Mechanisms m)
  {
    return frc2::cmd::RunEnd(
        [m]
        {
          m.intake.SetVoltage(hardware::kEjectVolts);
          m.conveyor.SetVoltage(hardware::kEjectConveyorVolts);
          m.bottomIndexer.SetVoltage(hardware::kEjectBottomVolts);
          m.topIndexer.SetVoltage(hardware::kEjectTopVolts);
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
          m.rack.SetPosition(hardware::kRackDeploy);
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
          m.rack.SetPosition(hardware::kRackSafe);
          m.intake.SetVoltage(hardware::kIntakeVolts);
          m.conveyor.SetVoltage(hardware::kStowConveyorVolts);
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
                 m.shooter.SetVelocity(*calibration ? ClampCalibrationFlywheel(table->GetNumber("FlywheelRPS", hardware::kHubRps))
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
    frc::Preferences::InitDouble("Intake/ShotRetractSpeedMps", hardware::kShotRetractSpeedMps);
    frc::Preferences::InitDouble("Intake/ShotHoldPositionMeters", hardware::kShotHoldPositionMetres);
    return frc2::cmd::RunEnd(
               [m, rumble, state]
               {
                 double now = Now();
                 bool feeding = state->gate.Calculate(m.shooter.VelocityControlActive(), m.shooter.Velocity(),
                                                      m.shooter.VelocitySetpoint(), now);
                 m.intake.SetVoltage(hardware::kIntakeVolts);
                 if (feeding)
                 {
                   Feed(m);
                   if (!state->feeding)
                     state->settings =
                         ShotIntakeSettings{frc::Preferences::GetDouble("Intake/ShotRetractSpeedMps", hardware::kShotRetractSpeedMps),
                                            frc::Preferences::GetDouble("Intake/ShotHoldPositionMeters", hardware::kShotHoldPositionMetres)};
                   m.rack.SetPositionDynamic(state->settings.holdPositionMeters,
                                             state->settings.retractSpeedMetersPerSecond, hardware::kShotIntakeAcceleration);
                 }
                 else
                 {
                   StopFeed(m);
                   m.rack.SetPosition(hardware::kRackDeploy);
                 }
                 state->feeding = feeding;
                 double phase = std::fmod(now - state->start, hardware::kRumblePeriodSeconds);
                 rumble(feeding ? hardware::kRumbleFeedIntensity : (phase < hardware::kRumbleFirstPulseEndSeconds || (phase >= hardware::kRumbleSecondPulseStartSeconds && phase < hardware::kRumbleSecondPulseEndSeconds)) ? hardware::kRumblePulseIntensity
                                                                                       : 0);
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
                   m.rack.SetPosition(hardware::kRackDeploy);
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
      double start = 0, lastFalse = 0, previous = 0, feedElapsed = 0, feedTarget = 0;
      bool started = false, feeding = false, transportActive = false, stationaryFerry = false;
      ShotIntakeSettings intakeSettings;
      VolleyFeedGate volley;
    };
    auto state = std::make_shared<State>();
    std::vector<frc2::Subsystem *> requirements{&m.hood, &m.shooter, &m.bottomIndexer, &m.topIndexer, &m.conveyor};
    if (options.agitate)
    {
      requirements.push_back(&m.rack);
      requirements.push_back(&m.intake);
      frc::Preferences::InitDouble("Intake/ShotRetractSpeedMps", hardware::kShotRetractSpeedMps);
      frc::Preferences::InitDouble("Intake/ShotHoldPositionMeters", hardware::kShotHoldPositionMetres);
    }
    if (options.ownDrive)
      requirements.push_back(&drive);
    auto action =
        frc2::cmd::RunEnd(
            [m, &drive, field, options, state]
            {
              double now = Now();
              if (state->transportActive)
                state->feedElapsed += now - state->previous;
              state->previous = now;
              auto pose = RobotPose(drive);
              // If motion prevents the moving solution from settling, hold
              // position for the remainder of this attempt instead of timing out.
              if (state->target.mode == ShotMode::Ferry && options.ferryTranslation &&
                  !state->started && now - state->start >= hardware::kHubReadyTimeoutSeconds)
                state->stationaryFerry = true;
              bool changed = false;
              if (state->target.mode == ShotMode::Ferry)
              {
                auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
                auto selected = FerryTarget(alliance, pose, field);
                changed = selected.position.X() != state->target.position.X() ||
                          selected.position.Y() != state->target.position.Y();
                state->target = selected;
              }
              const auto speeds = drive.Speeds();
              const bool movingFerry = state->target.mode == ShotMode::Ferry && options.ferryTranslation;
              auto solution = movingFerry ? CalculateMovingFerry(pose, state->target, field, speeds, options.simulation)
                                          : CalculateShot(pose, state->target, field, options.simulation);
              const bool durationValid = !options.autonomous ||
                                         (std::isfinite(state->feedTarget) && state->feedTarget > 0);
              bool calibration = options.calibration(), poseReady = options.poseReady() && durationValid,
                   force = options.forceFeed();
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
              auto raw =
                  EvaluateReadiness(poseReady, solution, speeds.vx.value(), speeds.vy.value(), speeds.omega.value(),
                                    pose.Rotation().Radians().value(), m.hood.Position(), m.shooter.Velocity(),
                                    movingFerry ? hardware::kMovingFerryMaxSpeed : hardware::kStationaryShotMaxSpeed, movingFerry ? hardware::kMovingFerryMaxAngularSpeed : hardware::kStationaryShotMaxAngleRadians);
              // Once a volley has started, tolerate modest flywheel variation
              // without repeatedly stopping and reversing the transport. Pose,
              // aim, motion, and valid-feedback checks still gate every tick.
              const bool spinningWithinVolleyBand = state->started && raw == Readiness::FlywheelSpinningUp &&
                                                    std::isfinite(m.shooter.Velocity()) && std::isfinite(solution.flywheelRps) &&
                                                    std::abs(m.shooter.Velocity() - solution.flywheelRps) <= hardware::kShotVelocityToleranceRps;
              bool readyCondition = !changed && !calibration &&
                                    (raw == Readiness::Ready || spinningWithinVolleyBand);
              if (!readyCondition)
                state->lastFalse = now;
              bool ready = options.autonomous
                               ? readyCondition && now - state->lastFalse >= kDebounceSeconds
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
              if (options.agitate)
              {
                m.intake.SetVoltage(hardware::kIntakeVolts);
                if (ready)
                {
                  if (!state->feeding)
                    state->intakeSettings = ShotIntakeSettings{
                        frc::Preferences::GetDouble("Intake/ShotRetractSpeedMps", hardware::kShotRetractSpeedMps),
                        frc::Preferences::GetDouble("Intake/ShotHoldPositionMeters", hardware::kShotHoldPositionMetres)};
                  m.rack.SetPositionDynamic(state->intakeSettings.holdPositionMeters,
                                            state->intakeSettings.retractSpeedMetersPerSecond, hardware::kShotIntakeAcceleration);
                }
                else
                  m.rack.SetPosition(hardware::kRackDeploy);
                frc::SmartDashboard::PutBoolean("Shooter/Agitating", ready);
              }
              state->feeding = ready;
              if (ready)
              {
                Feed(m);
                state->started = true;
              }
              else
                StopFeed(m);
              // During a reversal, voltage can indicate feed while the rollers
              // are still spinning in the collection direction.
              state->transportActive = ready && (!options.simulation || SimulatedTransportFeeding(m));
              options.readyFeedback(ready);
              options.feedFeedback(state->transportActive);
              frc::SmartDashboard::PutBoolean("Shooter/TransportActive", state->transportActive);
              double error = std::atan2(std::sin(solution.headingRadians - pose.Rotation().Radians().value()),
                                        std::cos(solution.headingRadians - pose.Rotation().Radians().value()));
              if (options.ownDrive)
              {
                const auto translation = movingFerry && poseReady && !state->stationaryFerry
                                             ? options.ferryTranslation()
                                             : frc::Translation2d{};
                drive.RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    units::meters_per_second_t{translation.X().value()}, units::meters_per_second_t{translation.Y().value()},
                    units::radians_per_second_t{(poseReady || force) ? std::clamp(error * hardware::kAutoAimHeadingGain, -hardware::kAutoAimMaxAngularRate, hardware::kAutoAimMaxAngularRate) : 0}, pose.Rotation()));
              }
              frc::SmartDashboard::PutBoolean("Shooter/Ready", ready);
              frc::SmartDashboard::PutBoolean("Shooter/Feeding", ready);
            },
            [m, &drive, options]
            {
              StopFeed(m);
              if (options.agitate)
              {
                m.rack.Stop();
                m.intake.Stop();
                frc::SmartDashboard::PutBoolean("Shooter/Agitating", false);
              }
              m.shooter.Stop();
              m.hood.Stop();
              if (options.ownDrive)
                drive.Stop();
              options.readyFeedback(false);
              options.feedFeedback(false);
              frc::SmartDashboard::PutBoolean("Shooter/TransportActive", false);
              frc::SmartDashboard::PutBoolean("Shooter/Ready", false);
              frc::SmartDashboard::PutBoolean("Shooter/Feeding", false);
            },
            frc2::Requirements{std::span<frc2::Subsystem *const>{requirements}})
            .BeforeStarting(
                [state, supplier, options]
                {
                  state->target = supplier();
                  state->feedTarget = options.feedDurationSeconds ? options.feedDurationSeconds() : options.feedSeconds;
                  state->start = state->lastFalse = state->previous = Now();
                  state->feedElapsed = 0;
                  state->started = state->feeding = state->transportActive = state->stationaryFerry = false;
                  state->volley.Reset(state->start);
                });
    if (options.autonomous)
      return std::move(action)
          .Until(
              [state, options]
              {
                return !std::isfinite(state->feedTarget) || state->feedTarget <= 0 ||
                       state->feedElapsed >= state->feedTarget ||
                       (state->transportActive && options.feedComplete && options.feedComplete()) ||
                       (!state->started && Now() - state->start >= options.readyTimeoutSeconds) ||
                       Now() - state->start >= std::max(options.totalTimeoutSeconds,
                                                        options.readyTimeoutSeconds + state->feedTarget);
              });
    return action;
  }
} // namespace season
