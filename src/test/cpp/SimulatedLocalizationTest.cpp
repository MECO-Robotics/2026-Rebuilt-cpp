#ifndef __FRC_ROBORIO__
#include "season/SimulatedLocalization.h"
#include <gtest/gtest.h>

TEST(SimulatedLocalization, TagVisibilityAnchorsTrackingAndResetRequiresNewAnchor) {
    using namespace units::literals;
    season::simulation::SimulatedLocalization sensors;
    const frc::AprilTagFieldLayout layout{
        std::vector<frc::AprilTag>{{1, frc::Pose3d{4_m, 2_m, 1_m,
            frc::Rotation3d{0_deg, 0_deg, 180_deg}}}}, 10_m, 8_m};
    const frc::Transform3d mount{0_m, 0_m, 1_m, frc::Rotation3d{}};
    const frc::Pose3d facing{2_m, 2_m, 0_m, frc::Rotation3d{}};
    const frc::Pose3d away{2_m, 2_m, 0_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}};
    sensors.Update(away, 1_s, layout, mount, mount);
    EXPECT_TRUE(sensors.limelight.observations.empty());
    EXPECT_TRUE(sensors.quest.observations.empty());
    sensors.Update(facing, 2_s, layout, mount, mount);
    ASSERT_EQ(sensors.limelight.observations.size(), 1u);
    ASSERT_EQ(sensors.quest.observations.size(), 1u);
    EXPECT_EQ(sensors.limelight.observations.front().pose, facing.ToPose2d());
    EXPECT_EQ(sensors.quest.observations.front().timestamp, 2_s);
    sensors.Update(away, 3_s, layout, mount, mount);
    EXPECT_TRUE(sensors.limelight.observations.empty());
    ASSERT_EQ(sensors.quest.observations.size(), 1u);
    EXPECT_EQ(sensors.quest.observations.front().pose, away.ToPose2d());
    sensors.Reset();
    sensors.Update(away, 4_s, layout, mount, mount);
    EXPECT_FALSE(sensors.quest.aligned);
    EXPECT_TRUE(sensors.quest.observations.empty());
    // Seeing the back of a tag cannot establish an absolute anchor.
    sensors.Update(frc::Pose3d{5_m, 2_m, 0_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}},
                   5_s, layout, mount, mount);
    EXPECT_TRUE(sensors.limelight.observations.empty());
    EXPECT_TRUE(sensors.quest.observations.empty());
}
TEST(SimulatedLocalization, QuestAprilTagsInitializeWithoutLimelight) {
    using namespace units::literals;
    season::simulation::SimulatedLocalization sensors;
    const frc::AprilTagFieldLayout layout{
        std::vector<frc::AprilTag>{
            {1, frc::Pose3d{4_m, 2_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}}},
            {2, frc::Pose3d{4_m, 2.5_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}}}}, 10_m, 8_m};
    const frc::Transform3d front{0_m, 0_m, 1_m, frc::Rotation3d{}};
    const frc::Transform3d rear{0_m, 0_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}};
    sensors.Update(frc::Pose3d{2_m, 2_m, 0_m, frc::Rotation3d{}}, 1_s, layout, rear, front);
    EXPECT_TRUE(sensors.limelight.observations.empty());
    EXPECT_TRUE(sensors.quest.aligned);
    EXPECT_EQ(sensors.quest.observations.size(), 1u);
}
#endif
