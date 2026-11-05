#pragma once
#include "base/Mechanism.h"
#include <array>
#include <memory>

namespace season {
// Shared output-shaft loads in N m, written by the fuel simulation.
struct FuelMotorLoads { std::array<double, 4> torque{}; };
}
namespace season::simulation {
// Binds season mechanism parameters to a lazily selected simulation provider.
base::MotorIOSelector::SimulationProvider MotorProvider(std::shared_ptr<FuelMotorLoads> loads);
}
