#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <frc/system/plant/DCMotor.h>
#include "base/Mechanism.h"

namespace base {
// All fields SI except gearRatio (motor turns/output turn) and unitsPerOutputRotation
// (1 for rotational mechanisms; drum circumference in metres for linear joints).
struct MechanismModel {
  double outputInertiaKgMetersSquared;
  double gearRatio;
  double unitsPerOutputRotation=1;
  double efficiency=1;
  double gravityTorqueNewtonMeters=0; // m*g*COM for arm; m*g*drumRadius for elevator
};
// Returns a copy preserving user PID/static-friction/limits. Explicitly opt-in; never
// replaces calibrated gains automatically. Ideal DC model excludes friction, compliance,
// current saturation and battery sag. DCMotor must represent the whole motor group.
inline Gains PretuneFeedforward(Gains gains, const frc::DCMotor& motors, MechanismModel model) {
  for(double value:{model.outputInertiaKgMetersSquared,model.gearRatio,
      model.unitsPerOutputRotation,model.efficiency,motors.R.value(),motors.Kt.value(),motors.Kv.value()})
    if(!std::isfinite(value) || value<=0) throw std::invalid_argument("Invalid mechanism model");
  if(model.efficiency>1 || !std::isfinite(model.gravityTorqueNewtonMeters))
    throw std::invalid_argument("Invalid model efficiency or gravity torque");
  const double radiansPerUnit=2*std::numbers::pi/model.unitsPerOutputRotation;
  const double voltsPerOutputTorque=motors.R.value()/(motors.Kt.value()*model.gearRatio*model.efficiency);
  gains.v=model.gearRatio*radiansPerUnit/motors.Kv.value();
  gains.a=model.outputInertiaKgMetersSquared*radiansPerUnit*voltsPerOutputTorque;
  gains.gravity=model.gravityTorqueNewtonMeters*voltsPerOutputTorque;
  return gains;
}
}
