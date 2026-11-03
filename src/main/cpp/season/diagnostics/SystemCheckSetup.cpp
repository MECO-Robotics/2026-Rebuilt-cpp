#include "season/mechanisms/Mechanisms.h"
#include "season/diagnostics/SystemCheckConfig.h"
#include <cmath>
#include <memory>
#include <frc/RobotController.h>

namespace season
{
  // Define bounded checks and their preflight dependencies.
  void ConfigureSystemChecks(base::SystemCheck &check, base::Drive &drive, base::PositionJoint &rack,
                             base::PositionJoint &hood, base::Flywheel &shooter,
                             std::vector<std::pair<std::string, base::Flywheel *>> rollers,
                             std::function<bool()> visionConnected)
  {
    auto rackStart = std::make_shared<double>(0);
    auto rackStarted = std::make_shared<bool>(false);
    // Connectivity is required before commanding mechanism motion.
    check.AddStage({"preflight",
                    diagnostics::kPreflightTimeoutSeconds,
                    [rackStarted]
                    { *rackStarted = false; },
                    [&rack, &hood, &shooter, rollers]
                    {
                      bool connected = rack.Inputs().connected && hood.Inputs().connected && shooter.Inputs().connected;
                      // Exercise each roller independently after preflight.
                      for (auto &[name, roller] : rollers)
                        connected = connected && roller && roller->Inputs().connected;
                      return base::CheckEvaluation{connected,
                                                   "Required mechanism controller connectivity",
                                                   {{"batteryVolts", frc::RobotController::GetBatteryVoltage().value()}}};
                    },
                    {},
                    diagnostics::kPreflightSettleSeconds});
    // Check unloaded forward velocity, not field driving accuracy.
    check.AddStage({"swerve-forward",
                    diagnostics::kDriveTimeoutSeconds,
                    [&drive]
                    {
                      drive.RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{diagnostics::kDriveSpeedMetresPerSecond}, units::meters_per_second_t{0},
                                                           units::radians_per_second_t{0}});
                    },
                    [&drive]
                    {
                      auto speeds = drive.Speeds();
                      return base::CheckEvaluation{std::abs(speeds.vx.value() - diagnostics::kDriveSpeedMetresPerSecond) < diagnostics::kDriveToleranceMetresPerSecond &&
                                                       std::abs(speeds.vy.value()) < diagnostics::kDriveToleranceMetresPerSecond,
                                                   "Unloaded forward velocity tracking",
                                                   {{"driveVx", speeds.vx.value()}, {"driveVy", speeds.vy.value()}}};
                    },
                    {"preflight"},
                    diagnostics::kMotionSettleSeconds});
    // A relative rack movement avoids assuming an uncalibrated absolute home reference.
    check.AddStage({"rack-motion",
                    diagnostics::kRackTimeoutSeconds,
                    [&rack, rackStart, rackStarted]
                    {
                      if (!*rackStarted)
                      {
                        *rackStart = rack.Position();
                        *rackStarted = true;
                      }
                      rack.SetPosition(*rackStart + diagnostics::kRackTravelMetres);
                    },
                    [&rack, rackStart]
                    {
                      return base::CheckEvaluation{
                          rack.Inputs().connected && std::abs(rack.Position() - (*rackStart + diagnostics::kRackTravelMetres)) < diagnostics::kRackToleranceMetres,
                          "Rack commanded relative to initial calibrated position",
                          {{"rackPosition", rack.Position()}, {"rackVelocity", rack.Velocity()}}};
                    },
                    {"preflight"},
                    diagnostics::kMotionSettleSeconds});
    // Check the hood against its measured position.
    check.AddStage({"hood-position",
                    diagnostics::kHoodTimeoutSeconds,
                    [&hood]
                    { hood.SetPosition(diagnostics::kHoodPositionRotations); },
                    [&hood]
                    {
                      return base::CheckEvaluation{
                          hood.Inputs().connected && std::abs(hood.Position() - diagnostics::kHoodPositionRotations) < diagnostics::kHoodToleranceRotations,
                          "Hood position tracking",
                          {{"hoodPosition", hood.Position()}, {"hoodVelocity", hood.Velocity()}}};
                    },
                    {"preflight"},
                    diagnostics::kMotionSettleSeconds});
    for (auto &[name, roller] : rollers)
    {
      if (!roller)
        throw std::invalid_argument("Null diagnostic roller");
      check.AddStage({name + "-roller",
                      diagnostics::kRollerTimeoutSeconds,
                      [roller]
                      { roller->SetVoltage(diagnostics::kRollerVoltage); },
                      [roller]
                      {
                        return base::CheckEvaluation{
                            roller->Inputs().connected && roller->Velocity() > diagnostics::kRollerMinimumRps,
                            "Positive roller motion at configured voltage",
                            {{"rollerVelocity", roller->Velocity()}, {"rollerCurrent", roller->Inputs().current}}};
                      },
                      {"preflight"},
                      diagnostics::kMotionSettleSeconds});
    }
    // Check unloaded shooter speed and record current.
    check.AddStage({"shooter-speed",
                    diagnostics::kShooterTimeoutSeconds,
                    [&shooter]
                    { shooter.SetVelocity(diagnostics::kShooterRps); },
                    [&shooter]
                    {
                      return base::CheckEvaluation{
                          shooter.Inputs().connected && std::abs(shooter.Velocity() - diagnostics::kShooterRps) < diagnostics::kShooterToleranceRps,
                          "Unloaded shooter speed tracking",
                          {{"shooterVelocity", shooter.Velocity()}, {"shooterCurrent", shooter.Inputs().current}}};
                    },
                    {"preflight"},
                    diagnostics::kMotionSettleSeconds});
    // A heartbeat does not prove pose acceptance or calibration.
    check.AddStage({"vision-connectivity",
                    diagnostics::kVisionTimeoutSeconds,
                    [] {},
                    [visionConnected]
                    {
                      bool connected = visionConnected && visionConnected();
                      return base::CheckEvaluation{
                          connected,
                          "Camera heartbeat connectivity only; accepted localization is not verified",
                          {{"visionConnected", connected ? 1. : 0.}}};
                    },
                    {},
                    diagnostics::kMotionSettleSeconds});
  }
} // namespace season
