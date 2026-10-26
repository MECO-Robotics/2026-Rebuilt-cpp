#ifndef __FRC_ROBORIO__
#include "season/SimulationPose.h"
#include <gtest/gtest.h>

TEST(SimulationPose, GroundReferenceRemovesBodyOffsetWithoutClampingMotionOrTilt) {
    using namespace units::literals;
    using season::simulation::FloorReferencedRobotPose;
    const auto groundHeight=.278_m;
    const frc::Pose3d grounded{2_m,3_m,groundHeight,frc::Rotation3d{}};
    const auto onFloor=FloorReferencedRobotPose(grounded,groundHeight);
    EXPECT_NEAR(onFloor.Z().value(),0,1e-12);
    EXPECT_DOUBLE_EQ(onFloor.X().value(),2);
    EXPECT_DOUBLE_EQ(onFloor.Y().value(),3);

    const auto elevated=FloorReferencedRobotPose(
        frc::Pose3d{2_m,3_m,groundHeight+.165_m,frc::Rotation3d{}},groundHeight);
    EXPECT_NEAR(elevated.Z().value(),.165,1e-12);

    // When overturned, the model's floor anchor is above the body center.
    // A world-Z subtraction (or forcing Z=0) would hide this geometry.
    const frc::Rotation3d upsideDown{180_deg,0_deg,0_deg};
    const auto inverted=FloorReferencedRobotPose(
        frc::Pose3d{2_m,3_m,.12_m,upsideDown},groundHeight);
    EXPECT_NEAR(inverted.Z().value(),.398,1e-12);
    EXPECT_EQ(inverted.Rotation(),upsideDown);
}
#endif
