#pragma once

#include "season/mechanisms/MechanismConfig.h"

namespace season::simulation
{
  // Inertia-only plants use output inertias in kg m².
  inline constexpr double kTransportRollerInertia = .0002;
  inline constexpr double kShooterInertia = 24 * .45359237 * .0254 * .0254;
  inline constexpr double kRackInertia = 15 * .45359237 * .0127 * .0127;
  inline constexpr double kHoodInertia = .01;

  inline double MotorInertia(hardware::MotorRole role)
  {
    return role == hardware::MotorRole::Shooter ? kShooterInertia : kTransportRollerInertia;
  }

  inline double JointInertia(hardware::JointRole role)
  {
    return role == hardware::JointRole::Rack ? kRackInertia : kHoodInertia;
  }


}
