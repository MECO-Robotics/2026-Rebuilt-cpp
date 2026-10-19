#pragma once
#include "base/Mechanism.h"
#include "season/MechanismConstants.h"
namespace season {
inline base::MotorConfig MotorConfigFor(const hardware::FlywheelConfig& c) {
  base::MotorConfig r;
  for(int i=0;i<c.motorCount;++i) {r.ids.push_back(c.ids[i]);r.reversed.push_back(c.inverted[i]);}
  r.gearRatio=c.gearRatio;r.currentLimit=static_cast<int>(c.currentLimitAmps);r.canBus=c.canBus;return r;
}
inline base::Gains GainsFor(const hardware::FlywheelGains& c) {
  base::Gains r;r.p=c.p;r.i=c.i;r.d=c.d;r.s=c.s;r.v=c.v;r.a=c.a;r.maxVelocity=1000;r.maxAcceleration=c.maxAcceleration>0?c.maxAcceleration:1000;r.tolerance=c.tolerance>0?c.tolerance:.01;return r;
}
inline base::Gains GainsFor(const hardware::JointGains& c) {
  base::Gains r;r.p=c.p;r.i=c.i;r.d=c.d;r.s=c.s;r.v=c.v;r.a=c.a;r.gravity=c.g;r.maxVelocity=c.maxVelocity;r.maxAcceleration=c.maxAcceleration;r.minimum=c.minPosition;r.maximum=c.maxPosition;r.tolerance=c.tolerance;return r;
}
inline base::Gains SimulationGainsFor(const hardware::JointGains& c) {
  auto gains=GainsFor(c);
  // The inertia-only plant has no gravitational load. Real gains remain unchanged.
  gains.gravity=0;
  return gains;
}
}
