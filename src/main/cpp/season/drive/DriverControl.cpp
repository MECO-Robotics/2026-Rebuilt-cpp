#include "season/drive/DriveBehaviorConfig.h"
#include "RobotContainer.h"
#include "season/detail/RuntimeAdapters.h"
#include "season/detail/RobotRuntime.h"
#include <frc/DriverStation.h>
#include <base/DriverInput.h>
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
  const auto input = base::ReadDriverInput(controller_.GetHID(), ntGamepad_.get(),
                                           season::drive::ntgamepadconfig::kFallbackToDriverStation);
  activeDriverInput_ = input.source;
  double x = deadband(input.WhileAuthorized(season::runtime::ObserveForwardInput(-input.axes.leftY))) * drive_.Config().maxSpeed;
  double y = deadband(-input.axes.leftX) * drive_.Config().maxSpeed;
  double omega = deadband(-input.axes.rightX) * drive_.Config().maxAngularRate;
  // Aim controls rotation and shooter targets while translation stays manual.
  if (aim && input.authorized)
  {
    const auto p = drive_.Pose();
    const auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed);
    const auto shot = season::CalculateDriverAim(alliance, p, field_, season::runtime::ShootingCalibration());
    hood_.SetPosition(shot.hoodRotations);
    shooter_.SetVelocity(shot.flywheelRps);
    const double error = std::remainder(shot.headingRadians - p.Rotation().Radians().value(), 2 * std::numbers::pi);
    omega = std::clamp(season::driveconfig::kAimHeadingGain * error, -drive_.Config().maxAngularRate, drive_.Config().maxAngularRate);
  }
  drive_.FieldDrive(x * driverHeadingOffset_.Cos() - y * driverHeadingOffset_.Sin(),
                    x * driverHeadingOffset_.Sin() + y * driverHeadingOffset_.Cos(), omega);
  runtime_->PublishDriverInput(input.nt, input.driverStationAxes.leftY, input.driverStationAxes.leftX,
                               input.driverStationAxes.rightX, x, y, omega);
}
