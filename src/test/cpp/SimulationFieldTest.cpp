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

#ifndef __FRC_ROBORIO__
// Exercises the packaged plant from the separate native-test source set.
// Accessing contact-load fields also checks that the package exports its matching
// public physics record layout.
TEST(SimulationField, LinkedAzimuthLoadVanishesWhenWheelIsLifted) {
    using namespace base::sim;
    SimulationWorld world(Backend::Full3D);
    BodySpec floor;floor.halfExtents={5,5,.05};floor.pose.position.z=-.05;world.CreateBody(floor);
    DrivePhysicsConfig config;config.mass=67.6;config.friction=2.1;
    DrivePhysics drive(world,config);drive.SetModuleVoltages(0,0,.3);
    for(int i=0;i<300;++i){drive.BeforePhysics(.001);world.Step(.001);drive.AfterPhysics(.001);}
    EXPECT_GT(std::abs(drive.Truth().modules[0].azimuthGroundTorque),.1);
    auto lifted=world.ReadBody(drive.Body());lifted.pose.position.z+=1;world.SetBodyState(drive.Body(),lifted);
    drive.BeforePhysics(.001);world.Step(.001);drive.AfterPhysics(.001);
    const auto module=drive.Truth().modules[0];
    EXPECT_DOUBLE_EQ(module.normalForce,0);
    EXPECT_DOUBLE_EQ(module.azimuthGroundTorque,0);
    EXPECT_DOUBLE_EQ(module.azimuthContactTwist,0);
}
#endif
