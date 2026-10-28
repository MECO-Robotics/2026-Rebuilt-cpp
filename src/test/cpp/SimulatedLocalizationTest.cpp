#ifndef __FRC_ROBORIO__
#include "season/SimulatedLocalization.h"
#include <gtest/gtest.h>
#include <limits>

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

TEST(SimulatedLocalization, FieldAndHeightBoundsGateObservations) {
    using namespace units::literals;
    const frc::AprilTagFieldLayout layout{std::vector<frc::AprilTag>{}, 10_m, 8_m};
    const auto make = [&](const frc::Pose3d& pose, bool fix) {
        return meco::sim::MakeIdealVisionObservation(pose, 5_s, layout, fix, .75_m, {.1, .2, .3});
    };
    auto inside = make(frc::Pose3d{10_m, 8_m, .75_m, frc::Rotation3d{}}, true);
    ASSERT_TRUE(inside);
    EXPECT_EQ(inside->stdDevs, (std::array<double, 3>{.1, .2, .3}));
    EXPECT_FALSE(make(frc::Pose3d{10.01_m, 8_m, 0_m, frc::Rotation3d{}}, true));
    EXPECT_FALSE(make(frc::Pose3d{1_m, 1_m, .76_m, frc::Rotation3d{}}, true));
    EXPECT_FALSE(make(frc::Pose3d{1_m, 1_m, 0_m, frc::Rotation3d{}}, false));
    EXPECT_FALSE(make(frc::Pose3d{1_m, 1_m, 0_m,
        frc::Rotation3d{0_rad, 0_rad, units::radian_t{std::numeric_limits<double>::quiet_NaN()}}}, true));
    EXPECT_THROW(meco::sim::MakeIdealVisionObservation(
        frc::Pose3d{}, 5_s, layout, true, .75_m,
        {std::numeric_limits<double>::quiet_NaN(), .1, .1}), std::invalid_argument);
}
#endif
