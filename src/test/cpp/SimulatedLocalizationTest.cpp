#ifndef __FRC_ROBORIO__
#include "season/SimulatedLocalization.h"
#include <gtest/gtest.h>

TEST(SimulatedLocalization, QuestKeepsItsFixAfterTagsDisappearAndResetClearsIt) {
    using namespace units::literals;
    const frc::AprilTagFieldLayout layout{
        std::vector<frc::AprilTag>{{1, frc::Pose3d{4_m, 2_m, 1_m,
            frc::Rotation3d{0_deg, 0_deg, 180_deg}}}}, 10_m, 8_m};
    const frc::Pose3d truth{2_m, 2_m, 0_m, frc::Rotation3d{}};
    const frc::Transform3d forward{0_m, 0_m, 1_m, frc::Rotation3d{}};
    const frc::Transform3d backward{0_m, 0_m, 1_m, frc::Rotation3d{0_deg, 0_deg, 180_deg}};
    season::simulation::SimulatedLocalization model;
    model.Update(truth, 2_s, layout, forward, backward);
    ASSERT_EQ(model.limelight.observations.size(), 1u);
    ASSERT_EQ(model.quest.observations.size(), 1u);
    EXPECT_EQ(model.quest.observations.front().timestamp, 2_s);
    model.Update(truth, 3_s, layout, backward, backward);
    EXPECT_TRUE(model.limelight.observations.empty());
    EXPECT_EQ(model.quest.observations.size(), 1u);
    model.Reset();
    model.Update(truth, 4_s, layout, backward, backward);
    EXPECT_TRUE(model.quest.observations.empty());
    EXPECT_FALSE(model.quest.aligned);
}

#endif
