#include "season/HubShift.h"
#include <gtest/gtest.h>
using namespace season;
TEST(HubShift,FirstAllianceUsesFirstFmsCharacterAndDefaultsBlue){
 EXPECT_EQ(FirstActiveAlliance("R"),Alliance::Red);EXPECT_EQ(FirstActiveAlliance("red"),Alliance::Red);
 EXPECT_EQ(FirstActiveAlliance("B"),Alliance::Blue);EXPECT_EQ(FirstActiveAlliance(""),Alliance::Blue);EXPECT_EQ(FirstActiveAlliance("?R"),Alliance::Blue);
}
TEST(HubShift,TransitionAndExactAlternatingBoundariesMatchJava){
 auto transition=ComputeHubShift(135,true,Alliance::Red,"R");EXPECT_EQ(transition.currentShift,ShiftState::Transition);EXPECT_DOUBLE_EQ(transition.remainingTime,10);EXPECT_FALSE(transition.active);
 auto first=ComputeHubShift(125,true,Alliance::Red,"R");EXPECT_EQ(first.currentShift,ShiftState::RedActive);EXPECT_DOUBLE_EQ(first.remainingTime,25);EXPECT_TRUE(first.active);
 auto second=ComputeHubShift(100,true,Alliance::Red,"R");EXPECT_EQ(second.currentShift,ShiftState::BlueActive);EXPECT_DOUBLE_EQ(second.remainingTime,25);EXPECT_FALSE(second.active);
 auto next=ComputeHubShift(75,true,Alliance::Red,"R");EXPECT_EQ(next.currentShift,ShiftState::RedActive);EXPECT_TRUE(next.active);
 auto zero=ComputeHubShift(-1,true,Alliance::Blue);EXPECT_DOUBLE_EQ(zero.matchTime,0);EXPECT_EQ(zero.currentShift,ShiftState::RedActive);EXPECT_FALSE(zero.active);
}
TEST(HubShift,NonTeleopUsesCountdownRemainderAndDoesNotUseFms){
 auto first=ComputeHubShift(20,false,Alliance::Blue,"R");EXPECT_EQ(first.currentShift,ShiftState::BlueActive);EXPECT_DOUBLE_EQ(first.remainingTime,20);EXPECT_TRUE(first.active);
 auto second=ComputeHubShift(25,false,Alliance::Blue);EXPECT_EQ(second.currentShift,ShiftState::RedActive);EXPECT_DOUBLE_EQ(second.remainingTime,0);EXPECT_FALSE(second.active);
 EXPECT_FALSE(ComputeHubShift(20,false,std::nullopt).active);
}
TEST(HubShift,ColorsUseStrictJavaThresholds){
 EXPECT_STREQ(ComputeHubShift(50,false,Alliance::Blue).matchTimeColor,"#FFFF00");EXPECT_STREQ(ComputeHubShift(50.01,false,Alliance::Blue).matchTimeColor,"#0000FF");
 EXPECT_STREQ(ComputeHubShift(20,false,Alliance::Blue).matchTimeColor,"#FF0000");EXPECT_STREQ(ComputeHubShift(20.01,false,Alliance::Blue).matchTimeColor,"#FFFF00");
 EXPECT_STREQ(ComputeHubShift(10,false,Alliance::Blue).shiftTimeColor,"#FF0000");EXPECT_STREQ(ComputeHubShift(10.01,false,Alliance::Blue).shiftTimeColor,"#00FF00");
 EXPECT_STREQ(ShiftStateName(ShiftState::BlueActive),"BLUE_ACTIVE");
}
