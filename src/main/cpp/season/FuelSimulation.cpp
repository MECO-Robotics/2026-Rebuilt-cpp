#include "season/FuelSimulation.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
namespace season {
namespace {
constexpr double diameter=5.5*.0254,radius=diameter/2;
FuelPoint World(FuelPoint local,Pose robot){double c=std::cos(robot.heading),s=std::sin(robot.heading);return {robot.x+local.x*c-local.y*s,robot.y+local.x*s+local.y*c,local.z};}
std::vector<double> Positions(const std::vector<FuelParticle>& particles){std::vector<double> values;for(auto& p:particles){values.push_back(p.position.x);values.push_back(p.position.y);values.push_back(p.position.z);}return values;}
std::vector<double> Positions(const std::vector<FuelPoint>& particles){std::vector<double> values;for(auto& p:particles){values.push_back(p.x);values.push_back(p.y);values.push_back(p.z);}return values;}
}
FuelModel::FuelModel(unsigned seed):seed_(seed),random_(seed){Reset();}
void FuelModel::Reset(){random_.seed(seed_);floor_.clear();airborne_.clear();stored_=scored_=launched_=pending_=burstIndex_=0;time_=0;lastBurst_=nextShot_=-1e9;}
void FuelModel::AddFloorFuel(Point point){floor_.push_back({{point.x,point.y,radius},{}});}
void FuelModel::SetStoredFuel(int count){stored_=std::clamp(count,0,54);}
double FuelModel::Random(double amplitude){return std::uniform_real_distribution<double>{-amplitude,amplitude}(random_);}
double FuelModel::LaunchPitch(double hood){double deflection=std::clamp(hood*2*kPi+16*kPi/180,15*kPi/180,45*kPi/180);return kPi/2-deflection;}
std::vector<FuelPoint> FuelModel::HopperCenters(int count,double extension,Pose robot){
 std::vector<FuelPoint> centers;count=std::clamp(count,0,54);double maxX=std::max(0.,extension)*std::cos(7.5*kPi/180);int zi=0;
 for(double z=.18+radius;z<=.65-radius+1e-9;z+=std::sqrt(3.)/2*diameter,++zi){int xi=0;for(double x=-.33+radius+(zi%2?radius:0);x<=maxX-radius+1e-9;x+=diameter,++xi)for(double y=-.34+radius+((zi+xi)%2?radius:0);y<=.34-radius+1e-9;y+=diameter){if(static_cast<int>(centers.size())==count)return centers;centers.push_back(World({x,y,z},robot));}}
 return centers;
}
void FuelModel::Launch(const FuelInputs& in,int index){
 double lateral=(index-1.5)*.08;FuelPoint position=World({-.19,lateral,kReleaseHeight},in.robot);
 double pitch=std::clamp(LaunchPitch(in.hoodRotations)+Random(kPi/180),kPi/4,75*kPi/180);
 double speed=std::abs(in.flywheelRps)*kProjectileSpeedPerRps*(1+Random(.01));
 double horizontal=speed*std::cos(pitch),dx=position.x-in.robot.x,dy=position.y-in.robot.y;
 airborne_.push_back({position,{horizontal*std::cos(in.robot.heading)+in.vx-in.omega*dy,horizontal*std::sin(in.robot.heading)+in.vy+in.omega*dx,speed*std::sin(pitch)}});++launched_;
}
void FuelModel::Step(double dt,const FuelInputs& in){
 if(!std::isfinite(dt)||dt<=0||dt>.25)return;
 time_+=dt;
 if(in.collecting&&in.rackMetres>.12&&stored_<54){double c=std::cos(in.robot.heading),s=std::sin(in.robot.heading);for(auto it=floor_.begin();it!=floor_.end()&&stored_<54;){double dx=it->position.x-in.robot.x,dy=it->position.y-in.robot.y,x=dx*c+dy*s,y=-dx*s+dy*c;if(x>=.28&&x<=.28+.20+radius&&std::abs(y)<=.35+radius){++stored_;it=floor_.erase(it);}else ++it;}}
 if(in.feeding&&std::abs(in.flywheelRps)>=5){if(!pending_&&time_-lastBurst_>=.20){pending_=4;burstIndex_=0;nextShot_=time_;lastBurst_=time_;}while(pending_&&time_>=nextShot_){if(!stored_){pending_=0;break;}--stored_;Launch(in,burstIndex_++);--pending_;if(pending_)nextShot_+=std::max(.01,.05+Random(.015));}}
 // Substeps resolve descending hub-height crossings even with coarse callers.
 int steps=std::max(1,static_cast<int>(std::ceil(dt/.005)));double h=dt/steps;
 for(int step=0;step<steps;++step){
  for(auto it=airborne_.begin();it!=airborne_.end();){auto previous=it->position;it->position.x+=it->velocity.x*h;it->position.y+=it->velocity.y*h;it->position.z+=it->velocity.z*h-.5*9.80665*h*h;it->velocity.z-=9.80665*h;
   Point hub=HubPosition(in.alliance);double height=72*.0254;bool crossing=previous.z>=height&&it->position.z<height&&it->velocity.z<0;double fraction=crossing?(previous.z-height)/(previous.z-it->position.z):0;double x=previous.x+(it->position.x-previous.x)*fraction,y=previous.y+(it->position.y-previous.y)*fraction;
   if(crossing&&std::abs(x-hub.x)<=.6096&&std::abs(y-hub.y)<=.6096){++scored_;double direction=in.alliance==Alliance::Blue?1:-1,angle=(direction>0?0:kPi)+Random(kPi/4);floor_.push_back({{hub.x+direction*.65,hub.y,radius},{3*std::cos(angle),3*std::sin(angle),0}});it=airborne_.erase(it);}
   else if(it->position.z<=radius){floor_.push_back({{it->position.x,it->position.y,radius},{it->velocity.x*.15,it->velocity.y*.15,0}});it=airborne_.erase(it);}else ++it;
  }
  for(auto& p:floor_){p.position.x+=p.velocity.x*h;p.position.y+=p.velocity.y*h;double attenuation=std::exp(-2*h);p.velocity.x*=attenuation;p.velocity.y*=attenuation;}
 }
}
FuelSimulation::FuelSimulation(Mechanisms mechanisms,base::Drive& drive):mechanisms_(mechanisms),drive_(drive){Reset();}
void FuelSimulation::Reset(){model_.Reset();
 // Deterministic neutral-zone grid, rather than claiming official Maple arena placement.
 for(double x=7.0;x<=9.5;x+=.25)for(double y=.75;y<=7.25;y+=.25)model_.AddFloorFuel({x,y});
}
void FuelSimulation::Periodic(double dt){auto pose=drive_.Pose();auto speed=drive_.Speeds();double heading=pose.Rotation().Radians().value(),c=std::cos(heading),s=std::sin(heading);
 FuelInputs input;input.robot={pose.X().value(),pose.Y().value(),heading};input.vx=speed.vx.value()*c-speed.vy.value()*s;input.vy=speed.vx.value()*s+speed.vy.value()*c;input.omega=speed.omega.value();input.rackMetres=mechanisms_.rack.Position();input.hoodRotations=mechanisms_.hood.Position();input.flywheelRps=mechanisms_.shooter.Velocity();input.collecting=mechanisms_.intake.Inputs().voltage<-1&&mechanisms_.conveyor.Inputs().voltage>1;input.feeding=mechanisms_.bottomIndexer.Inputs().voltage<-1&&mechanisms_.topIndexer.Inputs().voltage>1&&mechanisms_.conveyor.Inputs().voltage<-1;
 input.alliance=frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed?Alliance::Red:Alliance::Blue;model_.Step(dt,input);
 auto hopper=FuelModel::HopperCenters(model_.StoredFuel(),input.rackMetres,input.robot);
 frc::SmartDashboard::PutNumber("FieldSimulation/StoredFuelCount",model_.StoredFuel());frc::SmartDashboard::PutNumber("FieldSimulation/HopperFuelCount",hopper.size());frc::SmartDashboard::PutNumber("FieldSimulation/HopperFillRatio",hopper.empty()?0:static_cast<double>(hopper.size())/FuelModel::HopperCenters(54,input.rackMetres).size());
 frc::SmartDashboard::PutNumber("FieldSimulation/SuccessfulScoreCount",model_.SuccessfulScores());frc::SmartDashboard::PutNumber("FieldSimulation/LaunchedFuelCount",model_.LaunchedFuel());frc::SmartDashboard::PutNumber("FieldSimulation/FuelProjectileCount",model_.Projectiles().size());frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelPositionsXYZ",Positions(model_.FloorFuel()));frc::SmartDashboard::PutNumberArray("FieldSimulation/HopperFuelPositionsXYZ",Positions(hopper));frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelProjectilePositionsXYZ",Positions(model_.Projectiles()));
}
}
