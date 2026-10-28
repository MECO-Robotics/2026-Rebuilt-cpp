#ifndef __FRC_ROBORIO__
#include "meco/sim/SimulatedVision.h"
#include "meco/sim/SimulationPose.h"
#include <gtest/gtest.h>

TEST(SimulationVendordepGeometry, FloorOffsetRotatesWithBody) {
    using namespace units::literals;
    const frc::Rotation3d inverted{180_deg, 0_deg, 0_deg};
    const auto pose = meco::sim::FloorReferencedRobotPose(
        frc::Pose3d{2_m, 3_m, .12_m, inverted}, .278_m);
    EXPECT_NEAR(pose.Z().value(), .398, 1e-12);
    EXPECT_EQ(pose.Rotation(), inverted);
}

TEST(SimulationVendordepGeometry, VisibilityUsesMountRangeAndTagFacing) {
    using namespace units::literals;
    const frc::AprilTagFieldLayout layout{
        std::vector<frc::AprilTag>{
            {1, frc::Pose3d{4_m, 2_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}}},
            {2, frc::Pose3d{4_m, 2_m, 1_m, frc::Rotation3d{}}},
            {3, frc::Pose3d{4_m, 5_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}}}},
        10_m, 8_m};
    const frc::Pose3d robot{2_m, 2_m, 0_m, frc::Rotation3d{}};
    const frc::Transform3d mount{0_m, 0_m, 1_m, frc::Rotation3d{}};
    EXPECT_EQ(meco::sim::VisibleTags(robot, mount, layout, 60_deg, 50_deg, 2_m),
              std::vector<int>{1});
    EXPECT_TRUE(meco::sim::VisibleTags(robot, mount, layout, 60_deg, 50_deg, 1_m).empty());
    const frc::Transform3d rear{0_m, 0_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}};
    EXPECT_TRUE(meco::sim::VisibleTags(robot, rear, layout, 60_deg, 50_deg, 6_m).empty());
}
#endif
