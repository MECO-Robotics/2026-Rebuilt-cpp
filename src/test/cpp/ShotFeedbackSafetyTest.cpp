#include "season/Shooting.h"
#include <gtest/gtest.h>
#include <limits>

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
