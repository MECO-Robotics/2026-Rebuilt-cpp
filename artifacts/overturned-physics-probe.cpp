#include "base/sim/DrivePhysics.h"
#include <cmath>
#include <iostream>
using namespace base::sim;
int main(){
 for(bool upsideDown:{false,true}){
  SimulationWorld w(Backend::Full3D);BodySpec f;f.mass=0;f.halfExtents={100,100,.05};f.pose.position.z=-.05;w.CreateBody(f);
  DrivePhysicsConfig c;c.mass=53;c.inertia.z=6.883;c.driveRatio=5.890909;c.steerRatio=12.1;c.couplingRatio=4.909;c.friction=2.9;c.driveCurrentLimit=55;c.steerCurrentLimit=30;c.wheelInertia=.01;c.steerInertia=.01;
  DrivePhysics d(w,c);d.Reset({{0,0,.30},upsideDown?Quaternion{1,0,0,0}:Quaternion{0,0,0,1}});
  for(int k=0;k<1000;++k){auto t=d.Truth();for(int i=0;i<4;++i){double target=std::atan2(c.moduleLocations[i].x,-c.moduleLocations[i].y);double err=std::remainder(target-t.modules[i].steerAngle,2*std::acos(-1.));d.SetModuleVoltages(i,6,30.6*err-.612*t.modules[i].steerVelocity);}d.BeforePhysics(.004);w.Step(.004);d.AfterPhysics(.004);}
  auto t=d.Truth();double normal=0,vel=0;for(auto m:t.modules){normal+=m.normalForce;vel+=std::abs(m.wheelVelocity*c.wheelRadius)/4;}
  std::cout<<(upsideDown?"upside-down":"upright")<<" z="<<t.body.pose.position.z<<" yaw-rate="<<t.body.angularVelocity.z<<" avg-wheel-speed="<<vel<<" total-normal-force="<<normal<<"\n";
 }
}
