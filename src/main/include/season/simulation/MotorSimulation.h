#pragma once
#include "season/mechanisms/MechanismConfig.h"
#include "season/RobotGeometry.h"
#include <array>
#include <memory>

namespace season {
// Shared output-shaft loads in N m, written by the fuel simulation.
struct FuelMotorLoads { std::array<double, 4> torque{}; };
}
namespace season::simulation {
  // Inertia-only plants use output inertias in kg m².
  inline constexpr double kTransportRollerInertia = .0002;
  // Point-mass estimates: 24 lb at 1 in for the shooter and 15 lb at 0.5 in for the rack.
  inline constexpr double kShooterInertia = 24 * season::kKilogramsPerPound *
                                             season::kMetresPerInch * season::kMetresPerInch;
  inline constexpr double kRackInertia = 15 * season::kKilogramsPerPound *
                                          .5 * season::kMetresPerInch * .5 * season::kMetresPerInch;
  inline constexpr double kHoodInertia = .01;

  inline double MotorInertia(hardware::MotorRole role)
  {
    return role == hardware::MotorRole::Shooter ? kShooterInertia : kTransportRollerInertia;
  }

// Derives simulation entries on the first simulated motor request; the
// provider and resulting load callbacks retain the shared load state.
base::MotorIOSelector::SimulationProvider MotorProvider(std::shared_ptr<FuelMotorLoads> loads);
}
