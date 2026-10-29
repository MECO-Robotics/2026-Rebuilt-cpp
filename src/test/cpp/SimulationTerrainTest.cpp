#ifndef __FRC_ROBORIO__
#include "season/SimulationField.h"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

TEST(SimulationTerrain, SplitsBumpIntoUphillAndDownhillInEitherDirection) {
    using namespace season::simulation;
    const double y=kHubY-60*.0254;
    const base::FootprintPose start{kBlueHubX-1,y,0}, end{kBlueHubX+1,y,0};
    const auto cuts=RampBoundaries(start,end);
    ASSERT_EQ(cuts.size(),3u);
    auto pose=[&](double t) {return base::FootprintPose{start.x+(end.x-start.x)*t,y,0};};
    const auto uphill=RampTerrain(pose(cuts[0]),pose(cuts[1]));
    const auto downhill=RampTerrain(pose(cuts[1]),pose(cuts[2]));
    EXPECT_NEAR(uphill.grade,std::tan(15*std::numbers::pi/180),1e-9);
    EXPECT_NEAR(downhill.grade,-uphill.grade,1e-9);
    EXPECT_NEAR(RampTerrain(pose(cuts[1]),pose(cuts[0])).grade,-uphill.grade,1e-9);
    EXPECT_DOUBLE_EQ(RampTerrain(start,pose(cuts[0])).grade,0);
    const std::vector<base::FootprintPose> route{start,pose(cuts[0]),pose(cuts[1]),pose(cuts[2]),end};
    std::vector<base::FootprintTerrainSegment> terrain;
    for (std::size_t i=0;i+1<route.size();++i) terrain.push_back(RampTerrain(route[i],route[i+1]));
    // A 2 m/s² assumed drive capability cannot overcome the modeled slope;
    // a larger illustrative value demonstrates the profile is computable.
    EXPECT_TRUE(base::FootprintSpeedProfile(route,terrain,{1,2,2,2,1},0).empty());
    EXPECT_EQ(base::FootprintSpeedProfile(route,terrain,{1,4,4,2,1},0).size(),route.size());
    const auto obstacles=RampObstacles();
    EXPECT_EQ(obstacles.size(),8u);
    EXPECT_FALSE(base::SweptFootprintClear(start,end,{.86,.86},{0,0,kFieldLength,kFieldWidth},obstacles));
}
#endif
