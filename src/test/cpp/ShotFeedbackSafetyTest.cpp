#include "season/Shooting.h"
#include <gtest/gtest.h>
#include <limits>

TEST(DriverAim, ChoosesFerryOutsideAllianceZoneForBothAlliances) {
  const season::FieldGeometry field{16.541, 8.069, {}, {}};
  for (auto alliance : {frc::DriverStation::Alliance::kBlue, frc::DriverStation::Alliance::kRed}) {
    const double boundary = season::AllianceBoundary(alliance, field);
    const double direction = alliance == frc::DriverStation::Alliance::kBlue ? 1 : -1;
    const auto pose = [](double x) { return frc::Pose2d{units::meter_t{x}, units::meter_t{4}, frc::Rotation2d{}}; };
    EXPECT_EQ(season::DriverAimTarget(alliance, pose(boundary - direction * .01), field).mode, season::ShotMode::Hub);
    EXPECT_EQ(season::DriverAimTarget(alliance, pose(boundary + direction * .01), field).mode, season::ShotMode::Ferry);
    EXPECT_EQ(season::DriverAimTarget(alliance, pose(field.length / 2), field).mode, season::ShotMode::Ferry);
  }
}

TEST(DriverAim, FerryUsesStraightAllianceZoneShotForBothAlliances) {
  const season::FieldGeometry field{16.541, 8.069, {units::meter_t{4.6}, units::meter_t{1}},
                                    {units::meter_t{11.94}, units::meter_t{1}}};
  for (auto alliance : {frc::DriverStation::Alliance::kBlue, frc::DriverStation::Alliance::kRed}) {
    const double direction = alliance == frc::DriverStation::Alliance::kBlue ? -1 : 1;
    const double boundary = season::AllianceBoundary(alliance, field);
    for (bool simulation : {false, true}) {
      const frc::Pose2d near{units::meter_t{boundary - direction * 2}, units::meter_t{2},
                             frc::Rotation2d{units::radian_t{.7}}};
      const frc::Pose2d far{units::meter_t{boundary - direction * 4}, units::meter_t{2},
                            frc::Rotation2d{units::radian_t{-1.2}}};
      const auto first = season::CalculateDriverAim(alliance, near, field, simulation);
      const auto second = season::CalculateDriverAim(alliance, far, field, simulation);
      const double expectedHeading = direction < 0 ? (simulation ? season::kPi : 0) : (simulation ? 0 : season::kPi);
      EXPECT_NEAR(std::cos(first.headingRadians), std::cos(expectedHeading), 1e-9);
      EXPECT_NEAR(std::sin(first.headingRadians), std::sin(expectedHeading), 1e-9);
      EXPECT_NEAR(second.distanceMeters - first.distanceMeters, 2, 1e-9);
      EXPECT_GT(second.flywheelRps, first.flywheelRps);
    }
  }
}

TEST(ShotFeedbackSafety, InvalidFeedbackStopsAndRearmsFeed) {
  using namespace season;
  const auto shot = CalculateShot(frc::Pose2d{units::meter_t{3}, units::meter_t{0}, frc::Rotation2d{}}, frc::Translation2d{});
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_NE(EvaluateReadiness(true, shot, invalid, 0, 0, shot.headingRadians, shot.hoodRotations, shot.flywheelRps), Readiness::Ready);
    EXPECT_NE(EvaluateReadiness(true, shot, 0, 0, 0, invalid, shot.hoodRotations, shot.flywheelRps), Readiness::Ready);
    EXPECT_NE(EvaluateReadiness(true, shot, 0, 0, 0, shot.headingRadians, invalid, shot.flywheelRps), Readiness::Ready);
    EXPECT_NE(EvaluateReadiness(true, shot, 0, 0, 0, shot.headingRadians, shot.hoodRotations, invalid), Readiness::Ready);
    auto bad = shot; bad.flywheelRps = invalid;
    EXPECT_NE(EvaluateReadiness(true, bad, 0, 0, 0, shot.headingRadians, shot.hoodRotations, shot.flywheelRps), Readiness::Ready);
    VolleyFeedGate gate; gate.Reset(0);
    ASSERT_TRUE(gate.Calculate(true, 37, 37, .2));
    EXPECT_FALSE(gate.Calculate(true, invalid, 37, .21));
    EXPECT_FALSE(gate.Calculate(true, 37, 37, .22));
    EXPECT_TRUE(gate.Calculate(true, 37, 37, .4));
  }
}
