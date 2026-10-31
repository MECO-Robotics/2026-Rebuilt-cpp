#pragma once
#ifndef __FRC_ROBORIO__
#include "base/Mechanism.h"
#include <array>
#include <memory>

namespace season {
// Estimated opposing shaft torques, in N m. The fuel model owns these inputs;
// robot logic sees only the resulting motor measurements.
enum class LoadedMotor { Conveyor, Bottom, Top, Shooter };
struct FuelMotorLoads {
  std::array<double, 4> torque{};
};
std::unique_ptr<base::MotorIO> MakeLoadedMotorIO(const base::MotorConfig&, base::Vendor,
                                                double inertia, std::shared_ptr<FuelMotorLoads>, LoadedMotor);
}
#endif
