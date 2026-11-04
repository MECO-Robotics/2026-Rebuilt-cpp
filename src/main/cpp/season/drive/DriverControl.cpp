#include "RobotContainer.h"
#include "base/SimulationLatency.h"
#include "season/detail/RobotRuntime.h"
#include <frc/DriverStation.h>
#include <algorithm>
#include <cmath>
#include <numbers>

// Shape driver inputs before applying the driver-relative field frame.
void RobotContainer::DriveTeleop(bool aim)
{
  // Preserve MecoBase 2026.2.1's per-axis shaping before rotating the driver
  // translation frame. FieldDrive still owns alliance perspective and gating.
  const auto deadband = [](double value)
  { return std::abs(value) < season::driveconfig::kDriverDeadband ? 0. : std::copysign((std::abs(value) - season::driveconfig::kDriverDeadband) / (1 - season::driveconfig::kDriverDeadband), value); };
  const bool dsConnected = controller_.GetHID().IsConnected() && controller_.GetHID().GetAxisCount() >= 5;
  const double dsLeftY = dsConnected ? controller_.GetLeftY() : 0.0;
  const double dsLeftX = dsConnected ? controller_.GetLeftX() : 0.0;
  const double dsRightX = dsConnected ? controller_.GetRightX() : 0.0;
  const auto validAxis = [](double value)
  { return std::isfinite(value) && std::abs(value) <= 1.0; };
  const bool dsInputValid = dsConnected &&
                            validAxis(dsLeftY) && validAxis(dsLeftX) && validAxis(dsRightX);
  double rawLeftY = 0.0, rawLeftX = 0.0, rawRightX = 0.0;
  bool inputAvailable = false;
  activeDriverInput_ = "none";
  season::drive::NtGamepad::State ntGamepadState;
  if (ntGamepad_)
    ntGamepadState = ntGamepad_->Read();
  if (ntGamepad_ && ntGamepadState.accepted)
  {
    rawLeftY = ntGamepadState.leftY;
    rawLeftX = ntGamepadState.leftX;
    rawRightX = ntGamepadState.rightX;
    inputAvailable = true;
    activeDriverInput_ = "nt";
  }
  else if (dsInputValid && (!ntGamepad_ || season::drive::ntgamepadconfig::kFallbackToDriverStation))
  {
    rawLeftY = dsLeftY;
    rawLeftX = dsLeftX;
    rawRightX = dsRightX;
    inputAvailable = true;
    activeDriverInput_ = "driverstation";
  }
  const bool teleopAuthority = frc::DriverStation::IsTeleopEnabled() && !frc::DriverStation::IsEStopped();
  double x = deadband(base::simulation::ObserveJoystick(-rawLeftY)) * drive_.Config().maxSpeed;
  double y = deadband(-rawLeftX) * drive_.Config().maxSpeed;
  double omega = deadband(-rawRightX) * drive_.Config().maxAngularRate;
  if (!teleopAuthority || !inputAvailable)
    x = y = omega = 0.0;
  // Aim controls rotation and shooter targets while translation stays manual.
  if (aim && teleopAuthority && inputAvailable)
  {
    const auto p = drive_.Pose();
    const auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
    const auto shot = season::CalculateDriverAim(alliance, p, field_, simulationRuntime_->IsSimulation());
    hood_.SetPosition(shot.hoodRotations);
    shooter_.SetVelocity(shot.flywheelRps);
    const double error = std::remainder(shot.headingRadians - p.Rotation().Radians().value(), 2 * std::numbers::pi);
    omega = std::clamp(season::driveconfig::kAimHeadingGain * error, -drive_.Config().maxAngularRate, drive_.Config().maxAngularRate);
  }
  drive_.FieldDrive(x * driverHeadingOffset_.Cos() - y * driverHeadingOffset_.Sin(),
                    x * driverHeadingOffset_.Sin() + y * driverHeadingOffset_.Cos(), omega);
  simulationRuntime_->PublishDriverInput(ntGamepadState, dsLeftY, dsLeftX, dsRightX, x, y, omega);
}
