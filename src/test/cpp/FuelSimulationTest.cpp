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

TEST(FuelSimulation,ResetReproducesSeededBurstAndFlight){
 FuelModel model{8324};
 FuelInputs input;input.feeding=true;input.flywheelRps=37;input.hoodRotations=.02;
 auto run=[&]{model.SetStoredFuel(4);for(int i=0;i<12;++i)model.Step(.02,input);return model.Projectiles();};
 auto first=run();ASSERT_EQ(first.size(),4u);
 model.Reset();auto second=run();ASSERT_EQ(second.size(),first.size());
 for(size_t i=0;i<first.size();++i){
  EXPECT_DOUBLE_EQ(first[i].position.x,second[i].position.x);
  EXPECT_DOUBLE_EQ(first[i].position.y,second[i].position.y);
  EXPECT_DOUBLE_EQ(first[i].position.z,second[i].position.z);
  EXPECT_DOUBLE_EQ(first[i].velocity.z,second[i].velocity.z);
 }
}
