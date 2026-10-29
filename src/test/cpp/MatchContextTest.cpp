#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include "season/MatchContext.h"
TEST(MatchContext, OfficialBoundariesBothResultsAndAlliances) {
  for(const std::string data:{"R","B"}) for(auto own:{season::Alliance::Blue,season::Alliance::Red}) {
    for(double time:{140.,130.001,130.,105.001,105.,80.001,80.,55.001,55.,30.001,30.,0.}) {
      auto c=season::ComputeMatchContext(true,false,true,own,time,data);
      bool both=time>130||time<=30;
      bool odd=(time<=130&&time>105)||(time<=80&&time>55);
      bool blue=both||((data=="R")==odd);
      ASSERT_TRUE(c.blueActive);ASSERT_TRUE(c.redActive);ASSERT_TRUE(c.ownActive);
      EXPECT_EQ(*c.blueActive,blue);EXPECT_EQ(*c.redActive,both||!blue);
      EXPECT_EQ(*c.ownActive,own==season::Alliance::Blue?blue:(both||!blue));
      ASSERT_TRUE(c.inactiveFirstAlliance);ASSERT_TRUE(c.firstActiveAlliance);
      EXPECT_EQ(*c.inactiveFirstAlliance,data=="R"?season::Alliance::Red:season::Alliance::Blue);
      EXPECT_EQ(*c.firstActiveAlliance,data=="R"?season::Alliance::Blue:season::Alliance::Red);
      EXPECT_EQ(c.ownNextChange,own==season::Alliance::Blue?c.blueNextChange:c.redNextChange);
    }
  }
  auto c=season::ComputeMatchContext(true,false,true,season::Alliance::Blue,140,"R");
  EXPECT_DOUBLE_EQ(*c.redNextChange,10);EXPECT_DOUBLE_EQ(*c.blueNextChange,35);
  ASSERT_TRUE(c.ownNextChange);EXPECT_DOUBLE_EQ(*c.ownNextChange,35);
  c=season::ComputeMatchContext(true,false,true,season::Alliance::Blue,50,"R");
  EXPECT_DOUBLE_EQ(*c.blueNextChange,20);EXPECT_DOUBLE_EQ(*c.redNextChange,50);
}
TEST(MatchContext, MissingDataDoesNotInventAllianceOrEndgame) {
  auto c=season::ComputeMatchContext(true,false,true,std::nullopt,-1,"");
  EXPECT_FALSE(c.periodTime);EXPECT_FALSE(c.blueActive);EXPECT_FALSE(c.ownActive);
  c=season::ComputeMatchContext(true,false,true,std::nullopt,120,"?");
  EXPECT_FALSE(c.blueActive);EXPECT_FALSE(c.gameDataValid);
  EXPECT_FALSE(c.inactiveFirstAlliance);EXPECT_FALSE(c.firstActiveAlliance);EXPECT_FALSE(c.ownNextChange);
  c=season::ComputeMatchContext(true,false,true,std::nullopt,135,"");
  ASSERT_TRUE(c.blueActive);EXPECT_TRUE(*c.blueActive);EXPECT_FALSE(c.ownActive);
  c=season::ComputeMatchContext(true,true,false,season::Alliance::Red,20,"");
  ASSERT_TRUE(c.ownActive);EXPECT_TRUE(*c.ownActive);
  c=season::ComputeMatchContext(false,true,false,season::Alliance::Red,0,"R");
  ASSERT_TRUE(c.ownActive);EXPECT_FALSE(*c.ownActive);
}

TEST(MatchContext, DriverStationGameDataFlowsToHubOrder) {
  ASSERT_TRUE(HAL_Initialize(500,0));
  using DS=frc::sim::DriverStationSim;
  DS::ResetData();DS::SetDsAttached(true);DS::SetEnabled(true);DS::SetAutonomous(false);
  DS::SetAllianceStationId(HAL_AllianceStationID_kBlue1);DS::SetMatchTime(120);
  for(const std::string data:{"R","B",""}) {
    DS::SetGameSpecificMessage(data);DS::NotifyNewData();
    const auto c=season::ReadMatchContext();
    ASSERT_TRUE(c.alliance);EXPECT_EQ(*c.alliance,season::Alliance::Blue);
    EXPECT_EQ(c.phase,"shift1");EXPECT_EQ(c.gameDataValid,!data.empty());
    if(data.empty()) {EXPECT_FALSE(c.ownActive);EXPECT_FALSE(c.ownNextChange);EXPECT_FALSE(c.inactiveFirstAlliance);}
    else {ASSERT_TRUE(c.ownActive);ASSERT_TRUE(c.ownNextChange);ASSERT_TRUE(c.inactiveFirstAlliance);
      EXPECT_EQ(*c.ownActive,data=="R");EXPECT_EQ(*c.inactiveFirstAlliance,data=="R"?season::Alliance::Red:season::Alliance::Blue);
      EXPECT_DOUBLE_EQ(*c.ownNextChange,15);}
  }
  DS::SetEnabled(false);DS::NotifyNewData();DS::ResetData();
}
