#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include "base/sim/DrivePhysics.h"
#include <gtest/gtest.h>

TEST(SimulationField, StartsInsideFieldClearOfStructuresAndOverStartingLine) {
    using namespace season::simulation;
    for (bool red : {false,true}) {
        const auto spawn=DefaultSpawn(red);
        EXPECT_TRUE(IsClearStartingPose(spawn));
        // A square chassis sweeps its largest footprint at 45 degrees. The
        // default lane must permit turning, not merely an axis-aligned spawn.
        EXPECT_TRUE(IsClearStartingPose({spawn.Translation(),frc::Rotation2d{units::degree_t{45}}}));
        const double line=red?kFieldLength-kStartingLineX:kStartingLineX;
        EXPECT_LT(std::abs(spawn.X().value()-line),.43);
    }
    EXPECT_FALSE(IsClearStartingPose(frc::Pose2d{}));
    EXPECT_FALSE(IsClearStartingPose({units::meter_t{kBlueHubX},units::meter_t{kHubY},frc::Rotation2d{}}));
}

TEST(SimulationField, HubStopsPoweredChassisWhileWheelsMaySlip) {
    using namespace base::sim;
    using namespace season::simulation;
    SimulationWorld world(Backend::Full3D);
    BodySpec floor;floor.mass=0;floor.halfExtents={100,100,.05};floor.pose.position.z=-.05;
    world.CreateBody(floor);
    DrivePhysicsConfig config;config.mass=53;config.driveCurrentLimit=55;
    DrivePhysics drive(world,config);
    Arena arena(world,[&]{return drive.Truth().body;},ArenaScene::Empty);
    InstallField(arena);
    drive.Reset({{2.5,kHubY,.3},{}});
    for(size_t i=0;i<4;++i)drive.SetModuleVoltages(i,4,0);
    for(int i=0;i<1000;++i){drive.BeforePhysics(.004);world.Step(.004);drive.AfterPhysics(.004);}
    const auto truth=drive.Truth();
    // Actual Jolt contact must stop the body before the near hub face.
    EXPECT_GT(truth.body.pose.position.x,3.1);
    EXPECT_LT(truth.body.pose.position.x,kBlueHubX-.5969-.35);
    EXPECT_LT(std::abs(truth.body.linearVelocity.x),.1);
}
#endif

#ifndef __FRC_ROBORIO__
TEST(SimulationField, LinkedDriveModelKeepsRigidModuleHeightUnderLoad) {
    using namespace base::sim;
    for (double mass : {53.,106.}) {
        SimulationWorld world(Backend::Full3D);
        BodySpec floor;floor.mass=0;floor.halfExtents={100,100,.05};floor.pose.position.z=-.05;
        world.CreateBody(floor);
        DrivePhysicsConfig config;config.mass=mass;
        DrivePhysics drive(world,config);
        for(int i=0;i<300;++i){drive.BeforePhysics(.004);world.Step(.004);drive.AfterPhysics(.004);}
        EXPECT_NEAR(drive.Truth().body.pose.position.z,config.rideHeight,.001);
    }
}
#endif
