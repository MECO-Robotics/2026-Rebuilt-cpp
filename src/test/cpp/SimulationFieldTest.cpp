#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include "base/sim/DrivePhysics.h"
#include <gtest/gtest.h>

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
