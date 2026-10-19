#include <gtest/gtest.h>
#include "base/QuestNav.h"
using namespace units::literals;
TEST(QuestNav, SuppressesUninitializedAndAlignsMountingOffset) {
  base::QuestNavConfig config;
  config.robotToHeadset = frc::Transform3d{frc::Translation3d{0.5_m, 0_m, 0_m}, frc::Rotation3d{}};
  base::QuestNav quest{config};
  EXPECT_FALSE(quest.Update({frc::Pose3d{1.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1_s, true}, 1_s));
  auto observation = quest.Update({frc::Pose3d{1.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1.02_s, true},
      1.02_s, base::QuestNavObservation{frc::Pose2d{4_m, 3_m, frc::Rotation2d{90_deg}}, 1.02_s});
  ASSERT_TRUE(observation);
  EXPECT_NEAR(observation->pose.X().value(), 4, 1e-9);
  EXPECT_NEAR(observation->pose.Y().value(), 3, 1e-9);
  observation = quest.Update({frc::Pose3d{2.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1.04_s, true}, 1.04_s);
  ASSERT_TRUE(observation);
  EXPECT_NEAR(observation->pose.X().value(), 4, 1e-9);
  EXPECT_NEAR(observation->pose.Y().value(), 4, 1e-9);
  EXPECT_EQ(observation->timestamp, 1.04_s);
}
TEST(QuestNav, RejectsStaleLostAndRepeatedFramesAndRequiresFreshAlignment) {
  base::QuestNav quest;
  const frc::Pose3d raw{1_m, 1_m, 0_m, frc::Rotation3d{}};
  const base::QuestNavObservation absolute{frc::Pose2d{2_m, 2_m, frc::Rotation2d{}}, 1_s};
  EXPECT_FALSE(quest.Update({raw, 1_s, true}, 1.2_s, absolute));
  EXPECT_FALSE(quest.Update({raw, 1.2_s, false}, 1.2_s, absolute));
  EXPECT_FALSE(quest.Update({raw, 1.22_s, true}, 1.22_s, absolute));
  EXPECT_FALSE(quest.Initialized());
  ASSERT_TRUE(quest.ResetPose(absolute.pose));
  EXPECT_FALSE(quest.Update({raw, 1.22_s, true}, 1.22_s));
  EXPECT_TRUE(quest.Update({raw, 1.24_s, true}, 1.24_s));
  EXPECT_FALSE(quest.Connected(1.4_s));
  quest.InvalidateAlignment();
  EXPECT_FALSE(quest.Update({raw, 1.26_s, true}, 1.26_s));
}
