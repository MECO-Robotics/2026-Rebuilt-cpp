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

TEST(FuelSimulation,TypedFieldPosesPreserveMetresAndClearEmptySnapshots){
 auto instance=nt::NetworkTableInstance::Create();
 {
  FuelTelemetry telemetry{instance};
  auto floorTopic=instance.GetStructArrayTopic<frc::Pose3d>("/AdvantageScope/Simulation/FloorFuel");
  auto floor=floorTopic.Subscribe({});
  auto hopper=instance.GetStructArrayTopic<frc::Pose3d>("/AdvantageScope/Simulation/HopperFuel").Subscribe({});
  auto projectiles=instance.GetStructArrayTopic<frc::Pose3d>("/AdvantageScope/Simulation/Projectiles").Subscribe({});
  FuelModel model;model.AddFloorFuel({2,3});model.SetStoredFuel(1);
  FuelInputs input;input.feeding=true;input.flywheelRps=30;model.Step(.02,input);
  auto centers=FuelModel::HopperCenters(1,.3,{4,5,kPi/2});
  telemetry.Publish(model,centers,1000000);
  EXPECT_EQ(floorTopic.GetTypeString(),"struct:Pose3d[]");
  auto floorPoses=floor.Get();ASSERT_EQ(floorPoses.size(),1u);
  EXPECT_DOUBLE_EQ(floorPoses[0].X().value(),2);EXPECT_DOUBLE_EQ(floorPoses[0].Y().value(),3);
  EXPECT_DOUBLE_EQ(floorPoses[0].Z().value(),5.5*.0254/2);
  EXPECT_EQ(floorPoses[0].Rotation(),frc::Rotation3d{});
  auto hopperPoses=hopper.Get();ASSERT_EQ(hopperPoses.size(),1u);
  EXPECT_DOUBLE_EQ(hopperPoses[0].X().value(),centers[0].x);
  EXPECT_DOUBLE_EQ(hopperPoses[0].Y().value(),centers[0].y);
  auto projectilePoses=projectiles.Get();ASSERT_EQ(projectilePoses.size(),1u);
  EXPECT_DOUBLE_EQ(projectilePoses[0].Z().value(),model.Projectiles()[0].position.z);
  EXPECT_EQ(floor.GetAtomic().time,1000000);
  EXPECT_EQ(hopper.GetAtomic().time,1000000);
  EXPECT_EQ(projectiles.GetAtomic().time,1000000);
  model.Reset();telemetry.Publish(model,{},2000000);
  EXPECT_TRUE(floor.Get().empty());EXPECT_TRUE(hopper.Get().empty());EXPECT_TRUE(projectiles.Get().empty());
 }
 nt::NetworkTableInstance::Destroy(instance);
}
