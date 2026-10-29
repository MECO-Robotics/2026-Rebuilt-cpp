#pragma once
#include "season/Shooting.h"
#include <frc/DriverStation.h>
#include <optional>
#include <string>
#include <cmath>
namespace season {
struct MatchContext {
  bool enabled{}, autonomous{}, teleop{}, gameDataValid{};
  std::optional<Alliance> alliance, inactiveFirstAlliance, firstActiveAlliance;
  std::optional<double> periodTime;
  std::string gameData, phase{"unknown"};
  std::optional<bool> blueActive, redActive, ownActive;
  std::optional<double> blueNextChange, redNextChange, ownNextChange;
};
inline MatchContext ComputeMatchContext(bool enabled,bool autonomous,bool teleop,
    std::optional<Alliance> alliance,double time,const std::string& data) {
  MatchContext c;
  c.enabled=enabled;c.autonomous=autonomous;c.teleop=teleop;c.alliance=alliance;c.gameData=data;
  c.gameDataValid=data=="R"||data=="B";
  if(c.gameDataValid) {
    // FMS reports the alliance inactive first, not the alliance active first.
    c.inactiveFirstAlliance=data=="R"?Alliance::Red:Alliance::Blue;
    c.firstActiveAlliance=data=="R"?Alliance::Blue:Alliance::Red;
  }
  if(std::isfinite(time)&&time>=0) c.periodTime=time;
  if(!enabled) {c.phase="disabled";c.blueActive=false;c.redActive=false;}
  else if(autonomous) {
    c.phase="autonomous";c.blueActive=true;c.redActive=true;
    if(c.periodTime) c.blueNextChange=c.redNextChange=*c.periodTime;
  }
  else if(teleop&&c.periodTime) {
    int shift=0;
    if(time>130) c.phase="transition";
    else if(time>105) {c.phase="shift1";shift=1;}
    else if(time>80) {c.phase="shift2";shift=2;}
    else if(time>55) {c.phase="shift3";shift=3;}
    else if(time>30) {c.phase="shift4";shift=4;}
    else c.phase="endgame";
    if(!shift) {c.blueActive=true;c.redActive=true;}
    else if(c.gameDataValid) {
      bool blue=(data=="R")== (shift%2==1);
      c.blueActive=blue;c.redActive=!blue;
    }
    if(c.gameDataValid) {
      // Search upcoming boundaries for a real activity change, not merely a phase change.
      const double boundaries[]{130,105,80,55,30,0};
      for(double boundary:boundaries) if(boundary<time || boundary==0) {
        bool blueAfter=false,redAfter=false;
        if(boundary>0) {
          if(boundary==30) blueAfter=redAfter=true;
          else {
            int next=boundary==130?1:boundary==105?2:boundary==80?3:4;
            blueAfter=(data=="R")== (next%2==1);redAfter=!blueAfter;
          }
        }
        if(c.blueActive && !c.blueNextChange && blueAfter!=*c.blueActive) c.blueNextChange=time-boundary;
        if(c.redActive && !c.redNextChange && redAfter!=*c.redActive) c.redNextChange=time-boundary;
      }
    } else if(time<=30) c.blueNextChange=c.redNextChange=time;
  }
  if(alliance) {
    c.ownActive=*alliance==Alliance::Blue?c.blueActive:c.redActive;
    c.ownNextChange=*alliance==Alliance::Blue?c.blueNextChange:c.redNextChange;
  }
  return c;
}
inline std::string AllianceLabel(std::optional<Alliance> alliance) {
  return !alliance?"unknown":*alliance==Alliance::Red?"red":"blue";
}
inline MatchContext ReadMatchContext() {
  auto a=frc::DriverStation::GetAlliance();
  std::optional<Alliance> alliance;
  if(a) alliance=*a==frc::DriverStation::Alliance::kBlue?Alliance::Blue:Alliance::Red;
  return ComputeMatchContext(frc::DriverStation::IsEnabled(),frc::DriverStation::IsAutonomous(),
      frc::DriverStation::IsTeleop(),alliance,frc::DriverStation::GetMatchTime().value(),
      frc::DriverStation::GetGameSpecificMessage());
}
}
