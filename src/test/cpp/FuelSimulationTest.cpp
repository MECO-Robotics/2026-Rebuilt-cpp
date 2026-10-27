#include "season/FuelSimulation.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
using namespace season;
TEST(FuelSimulation,HopperPackingGrowsTowardPositiveXAndTransformsToField){
 auto stowed=FuelModel::HopperCenters(100,0),deployed=FuelModel::HopperCenters(100,.3);
 ASSERT_FALSE(stowed.empty());ASSERT_LE(deployed.size(),54u);
 auto maxX=[](const auto& points){return std::max_element(points.begin(),points.end(),[](auto a,auto b){return a.x<b.x;})->x;};
 EXPECT_LE(maxX(stowed),0);EXPECT_GT(maxX(deployed),maxX(stowed));
 auto transformed=FuelModel::HopperCenters(1,.3,{2,3,kPi/2});auto local=FuelModel::HopperCenters(1,.3);
 EXPECT_NEAR(transformed[0].x,2-local[0].y,1e-9);EXPECT_NEAR(transformed[0].y,3+local[0].x,1e-9);
 EXPECT_TRUE(FuelModel::HopperCenters(-1,.3).empty());
}
TEST(FuelSimulation,HopperFollowsTheFullVisualizationPose){
 // A raised, tilted chassis must carry the hopper with it, without applying
 // the robot transform twice. Check stowed and deployed packing alike.
 const frc::Pose3d robot{units::meter_t{4},units::meter_t{2},units::meter_t{.35},
   frc::Rotation3d{units::degree_t{12},units::degree_t{-20},units::degree_t{90}}};
 for(double extension : {0.,.3}){
  const auto local=FuelModel::HopperCenters(54,extension);
  const auto field=FuelModel::HopperCenters3d(54,extension,robot);
  ASSERT_EQ(field.size(),local.size());
  for(size_t i=0;i<local.size();++i){
   const frc::Pose3d ball{units::meter_t{field[i].x},units::meter_t{field[i].y},units::meter_t{field[i].z},frc::Rotation3d{}};
   const auto relative=ball.RelativeTo(robot);
   EXPECT_NEAR(relative.X().value(),local[i].x,1e-9);
   EXPECT_NEAR(relative.Y().value(),local[i].y,1e-9);
   EXPECT_NEAR(relative.Z().value(),local[i].z,1e-9);
  }
 }
}
TEST(FuelSimulation,CollectionRequiresDeployedRunningIntakeAndHasBoundedCapacity){
 FuelModel model;model.AddFloorFuel({.4,0});FuelInputs in;in.rackMetres=.3;
 model.Step(.02,in);EXPECT_EQ(model.StoredFuel(),0);
 in.collecting=true;in.rackMetres=0;model.Step(.02,in);EXPECT_EQ(model.StoredFuel(),0);
 in.rackMetres=.3;model.Step(.02,in);EXPECT_EQ(model.StoredFuel(),1);EXPECT_TRUE(model.FloorFuel().empty());
 model.SetStoredFuel(100);EXPECT_EQ(model.StoredFuel(),54);model.AddFloorFuel({.4,0});model.Step(.02,in);EXPECT_EQ(model.FloorFuel().size(),1u);
}
TEST(FuelSimulation,BurstStaggersAndStopsAtStoredFuelCount){
 FuelModel model;model.SetStoredFuel(3);FuelInputs in;in.feeding=true;in.flywheelRps=30;
 model.Step(.02,in);EXPECT_EQ(model.LaunchedFuel(),1);EXPECT_EQ(model.StoredFuel(),2);
 for(int i=0;i<15;++i)model.Step(.02,in);
 EXPECT_EQ(model.LaunchedFuel(),3);EXPECT_EQ(model.StoredFuel(),0);
 int before=model.LaunchedFuel();in.feeding=false;model.SetStoredFuel(10);for(int i=0;i<20;++i)model.Step(.02,in);EXPECT_EQ(model.LaunchedFuel(),before);
}
TEST(FuelSimulation,LaunchUsesMeasuredVelocityRotatedMuzzleAndInheritedMotion){
 FuelModel model;model.SetStoredFuel(1);FuelInputs in;in.robot={2,3,kPi/2};in.vx=1;in.vy=2;in.feeding=true;in.flywheelRps=4.9;
 model.Step(.02,in);EXPECT_EQ(model.LaunchedFuel(),0);
 in.flywheelRps=30;model.Step(.02,in);ASSERT_EQ(model.Projectiles().size(),1u);
 auto p=model.Projectiles()[0];EXPECT_NEAR(p.velocity.x,1,1e-9);EXPECT_GT(p.velocity.y,2);EXPECT_GT(p.position.z,kReleaseHeight);
 EXPECT_NEAR(p.position.x,2+.12+.02,1e-9);
}
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
TEST(FuelSimulation,IdealHubCrossingScoresAndReturnsFuelToFloor){
 FuelModel model;model.SetStoredFuel(1);auto hub=HubPosition(Alliance::Blue);
 FuelInputs in;in.alliance=Alliance::Blue;in.robot={hub.x-3+.19,hub.y,0};in.hoodRotations=.02;in.feeding=true;
 double pitch=FuelModel::LaunchPitch(in.hoodRotations),height=72*.0254-kReleaseHeight;
 in.flywheelRps=std::sqrt(9.80665*9/(2*std::pow(std::cos(pitch),2)*(3*std::tan(pitch)-height)))/kProjectileSpeedPerRps;
 model.Step(.02,in);in.feeding=false;
 for(int i=0;i<300;++i)model.Step(.02,in);
 EXPECT_EQ(model.SuccessfulScores(),1);EXPECT_TRUE(model.Projectiles().empty());EXPECT_EQ(model.FloorFuel().size(),1u);
 model.Reset();EXPECT_EQ(model.SuccessfulScores(),0);EXPECT_TRUE(model.FloorFuel().empty());
}
