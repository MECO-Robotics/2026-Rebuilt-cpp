#pragma once
#include <cmath>
#include <numbers>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <array>

namespace season
{
  inline constexpr double kRackAngleRadians = 7.5 * std::numbers::pi / 180;

  // The hopper slides horizontally as the intake rack extends, in metres.
  inline frc::Pose3d HopperPose(double extension)
  {
    return {units::meter_t{extension * std::cos(kRackAngleRadians)}, units::meter_t{0},
            units::meter_t{0}, frc::Rotation3d{}};
  }

  // The front camera rides the hopper, not the sloping intake rack.
  inline frc::Transform3d HopperCameraMount(double extension, const frc::Transform3d &hopperToCamera)
  {
    const auto camera = HopperPose(extension).TransformBy(hopperToCamera);
    return {camera.Translation(), camera.Rotation()};
  }
}


namespace season
{
  // Robot_Remy model_0..5 order; metres and output rotations, matching Java CAD frame.
  inline std::array<frc::Pose3d, 6> RemyComponentPoses(
      double extension, double hood, double flywheel, double climber = 0)
  {
    auto pivoted = [](double rotations, frc::Translation3d pivot)
    {
      const frc::Rotation3d rotation{units::radian_t{0}, units::radian_t{rotations * 2 * std::numbers::pi}, units::radian_t{0}};
      return frc::Pose3d{pivot - pivot.RotateBy(rotation), rotation};
    };
    const frc::Translation3d shooter{units::meter_t{-0.103310}, units::meter_t{0}, units::meter_t{0.423}};
    const auto slide = HopperPose(extension);
    const auto x = slide.X();
    return {pivoted(flywheel, shooter), pivoted(hood, shooter),
            frc::Pose3d{x, units::meter_t{0}, units::meter_t{-extension * std::sin(kRackAngleRadians)}, frc::Rotation3d{}},
            slide, slide, pivoted(-climber, frc::Translation3d{units::meter_t{-0.290}, units::meter_t{0}, units::meter_t{0.512}})};
  }
}
