#pragma once
#include <array>
#include <cmath>
#include <numbers>
#include <frc/geometry/Pose3d.h>

namespace season {
// Robot_Remy model_0..5 order; metres and output rotations, matching Java CAD frame.
inline std::array<frc::Pose3d, 6> RemyComponentPoses(
    double extension, double hood, double flywheel, double climber = 0) {
  auto pivoted = [](double rotations, frc::Translation3d pivot) {
    const frc::Rotation3d rotation{units::radian_t{0}, units::radian_t{rotations * 2 * std::numbers::pi}, units::radian_t{0}};
    return frc::Pose3d{pivot - pivot.RotateBy(rotation), rotation};
  };
  const frc::Translation3d shooter{units::meter_t{-0.103310}, units::meter_t{0}, units::meter_t{0.423}};
  const double angle = 7.5 * std::numbers::pi / 180;
  const units::meter_t x{extension * std::cos(angle)};
  const frc::Pose3d slide{x, units::meter_t{0}, units::meter_t{0}, frc::Rotation3d{}};
  return {pivoted(flywheel, shooter), pivoted(hood, shooter),
          frc::Pose3d{x, units::meter_t{0}, units::meter_t{-extension * std::sin(angle)}, frc::Rotation3d{}},
          slide, slide, pivoted(-climber, frc::Translation3d{units::meter_t{-0.290}, units::meter_t{0}, units::meter_t{0.512}})};
}
}
