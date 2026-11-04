#pragma once
#include <cmath>
#include <numbers>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <array>

namespace season
{
  // Fixed intake-rack slope relative to horizontal, used to resolve its extension vector.
  inline constexpr double kRackAngleRadians = 7.5 * std::numbers::pi / 180;
  // Cosine of the rack slope, giving the horizontal displacement per metre of extension.
  inline const double kRackAngleCos = std::cos(kRackAngleRadians);
  // Sine of the rack slope, giving the vertical displacement per metre of extension.
  inline const double kRackAngleSin = std::sin(kRackAngleRadians);
}

namespace season::geometry
{
  // Robot envelope including bumpers, used by navigation and simulated placement.
  inline constexpr double kLengthMetres = .86;
  inline constexpr double kWidthMetres = .86;
}

namespace season::hardware
{
  // X and Z coordinates in metres of the shooter pivot in the robot frame, respectively.
  inline constexpr double kShooterPivotXMetres = -0.103310, kShooterPivotZMetres = 0.423;
  // X and Z coordinates in metres of the climber pivot in the robot frame, respectively.
  inline constexpr double kClimberPivotXMetres = -0.290, kClimberPivotZMetres = 0.512;
}

namespace season
{

  // The hopper slides horizontally as the intake rack extends, in metres.
  inline frc::Pose3d HopperPose(double extension)
  {
    return {units::meter_t{extension * kRackAngleCos}, units::meter_t{0}, units::meter_t{0},
            frc::Rotation3d{}};
  }

  // The front camera rides the hopper, not the sloping intake rack.
  inline frc::Transform3d HopperCameraMount(double extension, const frc::Transform3d &hopperToCamera)
  {
    const auto camera = HopperPose(extension).TransformBy(hopperToCamera);
    return {camera.Translation(), camera.Rotation()};
  }
} // namespace season

namespace season
{
  // Robot_Remy model_0..5 order; metres and output rotations, matching Java CAD frame.
  inline std::array<frc::Pose3d, 6> RemyComponentPoses(double extension, double hood, double flywheel, double climber = 0)
  {
    auto pivoted = [](double rotations, frc::Translation3d pivot)
    {
      const frc::Rotation3d rotation{units::radian_t{0}, units::radian_t{rotations * 2 * std::numbers::pi},
                                     units::radian_t{0}};
      return frc::Pose3d{pivot - pivot.RotateBy(rotation), rotation};
    };
    const frc::Translation3d shooter{units::meter_t{hardware::kShooterPivotXMetres}, units::meter_t{0}, units::meter_t{hardware::kShooterPivotZMetres}};
    const auto slide = HopperPose(extension);
    const auto x = slide.X();
    return {
        pivoted(flywheel, shooter),
        pivoted(hood, shooter),
        frc::Pose3d{x, units::meter_t{0}, units::meter_t{-extension * kRackAngleSin}, frc::Rotation3d{}},
        slide,
        slide,
        pivoted(-climber, frc::Translation3d{units::meter_t{hardware::kClimberPivotXMetres}, units::meter_t{0}, units::meter_t{hardware::kClimberPivotZMetres}})};
  }
} // namespace season
