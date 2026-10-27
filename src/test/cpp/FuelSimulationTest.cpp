#include "season/FuelSimulation.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
using namespace season;
TEST(FuelSimulation,LaunchUsesPhysicsPoseAndFullMotionInsteadOfOdometry){
 FuelModel flat,physical;flat.SetStoredFuel(1);physical.SetStoredFuel(1);
 FuelInputs input;input.feeding=true;input.flywheelRps=30;
 constexpr double dt=.02,g=9.80665;
 flat.Step(dt,input);
 const auto reference=flat.Projectiles().at(0);
 const auto translation=[](FuelPoint p){return frc::Translation3d{units::meter_t{p.x},units::meter_t{p.y},units::meter_t{p.z}};};
 const FuelRobotState robot{
  {units::meter_t{4},units::meter_t{2},units::meter_t{.4},
   frc::Rotation3d{units::degree_t{15},units::degree_t{-20},units::degree_t{90}}},
  {1,2,.3},{.4,-.3,.5},{0,0,.30}};
 // Deliberately disagree with truth, as odometry can after reset or wheel slip.
 input.robot={20,30,-1};input.vx=10;input.physics=robot;
 physical.Step(dt,input);ASSERT_EQ(physical.Projectiles().size(),1u);
 const auto actual=physical.Projectiles()[0];
 const auto offset=(translation({-.19,-.12,kReleaseHeight})-translation(robot.centerOfMass)).RotateBy(robot.pose.Rotation());
 const FuelPoint r{offset.X().value(),offset.Y().value(),offset.Z().value()};
 const auto w=robot.angularVelocity;
 const FuelPoint inherited{robot.velocity.x+w.y*r.z-w.z*r.y,robot.velocity.y+w.z*r.x-w.x*r.z,robot.velocity.z+w.x*r.y-w.y*r.x};
 const auto initialVelocity=translation({reference.velocity.x,reference.velocity.y,reference.velocity.z+g*dt}).RotateBy(robot.pose.Rotation())+translation(inherited);
 const auto muzzle=robot.pose.Translation()+translation({-.19,-.12,kReleaseHeight}).RotateBy(robot.pose.Rotation());
 EXPECT_NEAR(actual.velocity.x,initialVelocity.X().value(),1e-9);
 EXPECT_NEAR(actual.velocity.y,initialVelocity.Y().value(),1e-9);
 EXPECT_NEAR(actual.velocity.z,initialVelocity.Z().value()-g*dt,1e-9);
 EXPECT_NEAR(actual.position.x,muzzle.X().value()+initialVelocity.X().value()*dt,1e-9);
 EXPECT_NEAR(actual.position.y,muzzle.Y().value()+initialVelocity.Y().value()*dt,1e-9);
 EXPECT_NEAR(actual.position.z,muzzle.Z().value()+initialVelocity.Z().value()*dt-.5*g*dt*dt,1e-9);
}
