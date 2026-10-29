#ifndef __FRC_ROBORIO__
#include "season/FuelSimulation.h"
#include "season/RobotVisualization.h"
#include <networktables/NetworkTableInstance.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
namespace season {
namespace {
constexpr double diameter=5.5*.0254,radius=diameter/2;
frc::Translation3d Translation(FuelPoint p){return {units::meter_t{p.x},units::meter_t{p.y},units::meter_t{p.z}};}
FuelPoint Point3d(const frc::Translation3d& p){return {p.X().value(),p.Y().value(),p.Z().value()};}
frc::Pose3d Pose3d(Pose p){return {units::meter_t{p.x},units::meter_t{p.y},units::meter_t{0},frc::Rotation3d{units::radian_t{0},units::radian_t{0},units::radian_t{p.heading}}};}
FuelPoint World(FuelPoint local,const frc::Pose3d& robot){return Point3d(robot.Translation()+Translation(local).RotateBy(robot.Rotation()));}
FuelRobotState RobotState(const FuelInputs& in){return in.physics.value_or(FuelRobotState{Pose3d(in.robot),{in.vx,in.vy,0},{0,0,in.omega},{}});}
// Four prescribed bumper boxes retain the open center of this robot.
const std::vector<meco::sim::ContactBox> kBumpers = [] {
 constexpr double half=.43, depth=.0889, bottom=.0127, top=bottom+2*.06985;
 std::vector<meco::sim::ContactBox> boxes;
 for(int side=0;side<4;++side) {
  const bool alongX=side<2; const double sign=side%2?1:-1;
  boxes.push_back({{alongX?sign*(half-depth/2):0,alongX?0:sign*(half-depth/2),(bottom+top)/2},
                   {alongX?depth/2:half,alongX?half:depth/2,(top-bottom)/2}});
 }
 return boxes;
}();
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
 return HopperCenters3d(count,extension,Pose3d(robot));
}
std::vector<FuelPoint> FuelModel::HopperCenters3d(int count,double extension,const frc::Pose3d& robot){
 std::vector<FuelPoint> centers;count=std::clamp(count,0,54);double maxX=std::max(0.,extension)*std::cos(7.5*kPi/180);int zi=0;
 for(double z=.18+radius;z<=.65-radius+1e-9;z+=std::sqrt(3.)/2*diameter,++zi){int xi=0;for(double x=-.33+radius+(zi%2?radius:0);x<=maxX-radius+1e-9;x+=diameter,++xi)for(double y=-.34+radius+((zi+xi)%2?radius:0);y<=.34-radius+1e-9;y+=diameter){if(static_cast<int>(centers.size())==count)return centers;centers.push_back(World({x,y,z},robot));}}
 return centers;
}
void FuelModel::Launch(const FuelInputs& in,int index){
 const auto robot=RobotState(in);
 double lateral=(index-1.5)*.08;FuelPoint position=World({-.19,lateral,kReleaseHeight},robot.pose);
 double pitch=std::clamp(LaunchPitch(in.hoodRotations)+Random(kPi/180),kPi/4,75*kPi/180);
 double speed=std::abs(in.flywheelRps)*kProjectileSpeedPerRps*(1+Random(.01));
 const auto muzzleVelocity=Point3d(Translation({speed*std::cos(pitch),0,speed*std::sin(pitch)}).RotateBy(robot.pose.Rotation()));
 const auto com=World(robot.centerOfMass,robot.pose);
 const double dx=position.x-com.x,dy=position.y-com.y,dz=position.z-com.z;
 const auto& w=robot.angularVelocity;
 airborne_.push_back({position,{muzzleVelocity.x+robot.velocity.x+w.y*dz-w.z*dy,
                               muzzleVelocity.y+robot.velocity.y+w.z*dx-w.x*dz,
                               muzzleVelocity.z+robot.velocity.z+w.x*dy-w.y*dx}});++launched_;
}
void FuelModel::Step(double dt,const FuelInputs& in){
 if(!std::isfinite(dt)||dt<=0||dt>.25)return;
 time_+=dt;
 if(in.collecting&&in.rackMetres>.12&&stored_<54){const auto robot=RobotState(in).pose;for(auto it=floor_.begin();it!=floor_.end()&&stored_<54;){const auto local=(Translation(it->position)-robot.Translation()).RotateBy(-robot.Rotation());double x=local.X().value(),y=local.Y().value();if(x>=.28&&x<=.28+.20+radius&&std::abs(y)<=.35+radius){++stored_;it=floor_.erase(it);}else ++it;}}
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
  meco::sim::ResolveParticleContacts(floor_,airborne_,RobotState(in),radius,.25,.30,kBumpers);
  // A bumper or ball may lift resting fuel back into ballistic motion.
  for(auto it=floor_.begin();it!=floor_.end();){if(it->position.z>radius+1e-6||it->velocity.z>1e-6){airborne_.push_back(*it);it=floor_.erase(it);}else{it->position.z=radius;it->velocity.z=0;++it;}}
 }
}
FuelSimulation::FuelSimulation(Mechanisms mechanisms,base::Drive& drive):mechanisms_(mechanisms),drive_(drive){
 auto nt=nt::NetworkTableInstance::GetDefault();
 robotPosePublisher_=nt.GetStructTopic<frc::Pose3d>("/Visualization/RobotRemy/RobotPose").Publish();
 componentsPublisher_=nt.GetStructArrayTopic<frc::Pose3d>("/Visualization/RobotRemy/ComponentPoses").Publish();
 fuelPublisher_=nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelPositions").Publish();
 hopperPublisher_=nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/HopperFuelPositions").Publish();
 projectilesPublisher_=nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelProjectilePositions").Publish();
 Reset();}
void FuelSimulation::Reset(){model_.Reset();
 // Deterministic neutral-zone grid, rather than claiming official Maple arena placement.
 for(double x=7.0;x<=9.5;x+=.25)for(double y=.75;y<=7.25;y+=.25)model_.AddFloorFuel({x,y});
}
void FuelSimulation::Periodic(double dt, std::optional<FuelRobotState> physics){auto pose=drive_.Pose();auto speed=drive_.Speeds();double heading=pose.Rotation().Radians().value(),c=std::cos(heading),s=std::sin(heading);
 FuelInputs input;input.robot={pose.X().value(),pose.Y().value(),heading};input.vx=speed.vx.value()*c-speed.vy.value()*s;input.vy=speed.vx.value()*s+speed.vy.value()*c;input.omega=speed.omega.value();input.rackMetres=mechanisms_.rack.Position();input.hoodRotations=mechanisms_.hood.Position();input.flywheelRps=mechanisms_.shooter.Velocity();input.collecting=mechanisms_.intake.Inputs().voltage<-1&&mechanisms_.conveyor.Inputs().voltage>1;input.feeding=mechanisms_.bottomIndexer.Inputs().voltage<-1&&mechanisms_.topIndexer.Inputs().voltage>1&&mechanisms_.conveyor.Inputs().voltage<-1;
 input.physics=physics;
 input.alliance=frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed?Alliance::Red:Alliance::Blue;model_.Step(dt,input);
 const auto robotPose=RobotState(input).pose;
 robotPosePublisher_.Set(robotPose);
 componentsPublisher_.Set(RemyComponentPoses(input.rackMetres,input.hoodRotations,mechanisms_.shooter.Position()));
 auto hopper=FuelModel::HopperCenters3d(model_.StoredFuel(),input.rackMetres,robotPose);
 std::vector<frc::Pose3d> fuel, hopperPoses, projectiles;
 auto ballPose=[](FuelPoint p){return frc::Pose3d{units::meter_t{p.x},units::meter_t{p.y},units::meter_t{p.z},frc::Rotation3d{}};};
 for(const auto& p:model_.FloorFuel()) fuel.push_back(ballPose(p.position));
 for(const auto& p:hopper) hopperPoses.push_back(ballPose(p));
 fuel.insert(fuel.end(),hopperPoses.begin(),hopperPoses.end());
 for(const auto& p:model_.Projectiles()) projectiles.push_back(ballPose(p.position));
 fuelPublisher_.Set(fuel);hopperPublisher_.Set(hopperPoses);projectilesPublisher_.Set(projectiles);
 frc::SmartDashboard::PutNumber("FieldSimulation/StoredFuelCount",model_.StoredFuel());frc::SmartDashboard::PutNumber("FieldSimulation/HopperFuelCount",hopper.size());frc::SmartDashboard::PutNumber("FieldSimulation/HopperFillRatio",hopper.empty()?0:static_cast<double>(hopper.size())/FuelModel::HopperCenters(54,input.rackMetres).size());
 auto fuelPositions=Positions(model_.FloorFuel());const auto hopperPositions=Positions(hopper);fuelPositions.insert(fuelPositions.end(),hopperPositions.begin(),hopperPositions.end());
 frc::SmartDashboard::PutNumber("FieldSimulation/SuccessfulScoreCount",model_.SuccessfulScores());frc::SmartDashboard::PutNumber("FieldSimulation/LaunchedFuelCount",model_.LaunchedFuel());frc::SmartDashboard::PutNumber("FieldSimulation/FuelProjectileCount",model_.Projectiles().size());frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelPositionsXYZ",fuelPositions);frc::SmartDashboard::PutNumberArray("FieldSimulation/HopperFuelPositionsXYZ",hopperPositions);frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelProjectilePositionsXYZ",Positions(model_.Projectiles()));
}
}

#endif
