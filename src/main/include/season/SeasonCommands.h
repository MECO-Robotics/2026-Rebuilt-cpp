#pragma once
#include "base/Mechanism.h"
#include "base/Drive.h"
#include "season/Shooting.h"
#include <functional>
namespace season {
struct Mechanisms {
  base::PositionJoint& rack;
  base::Flywheel& intake;
  base::Flywheel& conveyor;
  base::Flywheel& bottomIndexer;
  base::Flywheel& topIndexer;
  base::PositionJoint& hood;
  base::Flywheel& shooter;
};
frc2::CommandPtr Acquire(Mechanisms m);
frc2::CommandPtr Eject(Mechanisms m);
frc2::CommandPtr Deploy(Mechanisms m);
frc2::CommandPtr Stow(Mechanisms m);
// One-shot presets intentionally retain mechanism control after command ends.
frc2::CommandPtr ShooterPreset(Mechanisms m, double hoodRotations, double flywheelRps);
frc2::CommandPtr ShooterIdle(Mechanisms m);
frc2::CommandPtr DriverHubPreset(Mechanisms m);
frc2::CommandPtr AgitateAndFeed(Mechanisms m, std::function<void(double)> rumble = [](double){});
struct CoordinatedShotOptions {
  std::function<bool()> poseReady = []{return true;};
  std::function<bool()> forceFeed = []{return false;};
  std::function<bool()> calibration = []{return false;};
  std::function<double()> calibrationHood = []{return 0.;};
  std::function<double()> calibrationRps = []{return 30.;};
  std::function<void(bool)> readyFeedback = [](bool){};
  bool autonomous = false, simulation = false;
  // Markers share the path drivetrain and must not claim or command it.
  bool ownDrive = true;
  double readyTimeoutSeconds=2, feedSeconds=1;
};
frc2::CommandPtr CoordinatedShot(Mechanisms m, base::Drive& drive, FieldGeometry field,
                               std::function<ShotTarget()> target, CoordinatedShotOptions options = {});
}
