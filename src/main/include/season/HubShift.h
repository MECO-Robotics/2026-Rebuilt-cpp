#pragma once
#include "season/Shooting.h"
#include <cmath>
#include <optional>
#include <string_view>
namespace season {
enum class ShiftState { BlueActive,RedActive,Transition };
struct ShiftInfo {
 double matchTime,remainingTime;
 bool active;
 ShiftState currentShift;
 const char* matchTimeColor;
 const char* shiftTimeColor;
};
inline Alliance FirstActiveAlliance(std::string_view gameData){
 if(!gameData.empty()&&(gameData.front()=='R'||gameData.front()=='r'))return Alliance::Red;
 return Alliance::Blue;
}
// Exact Java HubShiftUtil timing, including its 25-second endgame cycle.
// Negative/unavailable DS time becomes zero. No official-rule correction applied.
inline ShiftInfo ComputeHubShift(double matchTime,bool teleop,std::optional<Alliance> alliance,std::string_view gameData={}){
 if(!std::isfinite(matchTime)||matchTime<0)matchTime=0;
 double remaining;
 ShiftState state;
 bool active=false;
 if(teleop&&matchTime>125){state=ShiftState::Transition;remaining=matchTime-125;}
 else{
  double periodTime=teleop?125-matchTime:matchTime;
  remaining=teleop?25-std::fmod(periodTime,25):std::fmod(matchTime,25);
  int period=static_cast<int>(periodTime/25);
  Alliance first=teleop?FirstActiveAlliance(gameData):Alliance::Blue;
  Alliance current=period%2==0?first:(first==Alliance::Blue?Alliance::Red:Alliance::Blue);
  state=current==Alliance::Blue?ShiftState::BlueActive:ShiftState::RedActive;
  active=alliance&&*alliance==current;
 }
 return {matchTime,remaining,active,state,matchTime>50?"#0000FF":matchTime>20?"#FFFF00":"#FF0000",remaining>10?"#00FF00":"#FF0000"};
}
inline const char* ShiftStateName(ShiftState state){
 switch(state){case ShiftState::BlueActive:return "BLUE_ACTIVE";case ShiftState::RedActive:return "RED_ACTIVE";case ShiftState::Transition:return "TRANSITION";}return "TRANSITION";
}
}
