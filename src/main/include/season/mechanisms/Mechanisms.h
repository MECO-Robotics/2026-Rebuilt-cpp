#pragma once
#include "base/Mechanism.h"
#include "season/mechanisms/MechanismConfig.h"
#include "season/mechanisms/MechanismBehaviorConfig.h"
#include "season/shooting/Shooting.h"
#include "base/Drive.h"
#include <functional>
#include "base/SystemCheck.h"
#include <utility>
#include <units/voltage.h>



namespace season
{
  // Non-owning references shared by season command factories.
  struct Mechanisms
  {
    base::PositionJoint &rack;
    base::Flywheel &intake;
    base::Flywheel &conveyor;
    base::Flywheel &bottomIndexer;
    base::Flywheel &topIndexer;
    base::PositionJoint &hood;
    base::Flywheel &shooter;
  };
  // Voltage alone does not establish roller direction during a reversal.
  // Used by desktop fuel simulation and its matching feed-time feedback.
  inline bool SimulatedTransportFeeding(Mechanisms m)
  {
    return m.bottomIndexer.Inputs().voltage < -simulationconfig::kSimTransportVoltageThreshold &&
           m.topIndexer.Inputs().voltage > simulationconfig::kSimTransportVoltageThreshold &&
           m.conveyor.Inputs().voltage < -simulationconfig::kSimTransportVoltageThreshold &&
           m.bottomIndexer.Velocity() < 0 && m.topIndexer.Velocity() > 0 && m.conveyor.Velocity() < 0;
  }
  // Full-hopper feed duration from robot preferences, with the desktop model's
  // configured values as simulation defaults. Zero means unconfigured.
  double ConfiguredFeedDurationSeconds();
  // Acquisition commands own and stop their participating mechanisms.
  frc2::CommandPtr Acquire(Mechanisms m);
  frc2::CommandPtr Eject(Mechanisms m);
  frc2::CommandPtr Deploy(Mechanisms m);
  frc2::CommandPtr Stow(Mechanisms m);
  // One-shot presets intentionally retain mechanism control after command ends.
  frc2::CommandPtr ShooterPreset(Mechanisms m, double hoodRotations, double flywheelRps);
  frc2::CommandPtr ShooterIdle(Mechanisms m);
  frc2::CommandPtr DriverHubPreset(Mechanisms m);
  frc2::CommandPtr AgitateAndFeed(Mechanisms m, std::function<void(double)> rumble = [](double) {});
  // Suppliers are sampled by the running shot command.
  struct CoordinatedShotOptions
  {
    std::function<bool()> poseReady = []
    { return true; };
    std::function<bool()> forceFeed = []
    { return false; };
    std::function<bool()> calibration = []
    { return false; };
    std::function<double()> calibrationHood = []
    { return 0.; };
    std::function<double()> calibrationRps = []
    { return shotconfig::kHubRps; };
    std::function<void(bool)> readyFeedback = [](bool) {};
    // In simulation this reports measured transport activity, not just intent.
    std::function<void(bool)> feedFeedback = [](bool) {};
    bool autonomous = false, simulation = false;
    // Run the intake and retract the rack while a ready volley feeds.
    bool agitate = false;
    // Markers share the path drivetrain and must not claim or command it.
    bool ownDrive = true;
    double readyTimeoutSeconds = shotconfig::kAutoReadyTimeoutSeconds,
           feedSeconds = shotconfig::kAutoFeedSeconds,
           totalTimeoutSeconds = shotconfig::kAutoTotalTimeoutSeconds;
    // Optional duration sampled once when the command starts.
    std::function<double()> feedDurationSeconds;
    // Optional feedback-based early completion after feed has started.
    std::function<bool()> feedComplete;
    // Field-relative translation in metres per second while aiming a moving ferry.
    std::function<frc::Translation2d()> ferryTranslation;
  };
  // Autonomous shots end on feed completion or a bounded timeout.
  frc2::CommandPtr CoordinatedShot(Mechanisms m, base::Drive &drive, FieldGeometry field,
                                   std::function<ShotTarget()> target, CoordinatedShotOptions options = {});
} // namespace season

namespace season
{
  // Register unloaded pit checks only. Caller supplies ALL motors to neutralize in manager callback.
  // Shooter intentionally runs at 10 rps; no practice-shot/feed sequence is registered.
  void ConfigureSystemChecks(base::SystemCheck &check, base::Drive &drive, base::PositionJoint &rack,
                             base::PositionJoint &hood, base::Flywheel &shooter,
                             std::vector<std::pair<std::string, base::Flywheel *>> rollers,
                             std::function<bool()> visionConnected);
} // namespace season
