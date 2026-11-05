#pragma once
#include "base/Mechanism.h"
#include <array>
#include <memory>

namespace season {
// Shared output-shaft loads in N m, written by the fuel simulation.
struct FuelMotorLoads { std::array<double, 4> torque{}; };
}
namespace season::simulation {
// Derives simulation entries on the first simulated motor request; the
// provider and resulting load callbacks retain the shared load state.
base::MotorIOSelector::SimulationProvider MotorProvider(std::shared_ptr<FuelMotorLoads> loads);
}
