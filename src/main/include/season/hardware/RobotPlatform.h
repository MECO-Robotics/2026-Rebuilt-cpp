#pragma once
#include "base/Mechanism.h"
#include "base/RuntimeTuning.h"
#include "base/Drive.h"
namespace season::hardware
{
// Automatic uses simulation-derived gains; Configured uses robot gains everywhere.
inline constexpr auto kSimulationTuning = base::SimulationTuning::Automatic;
base::MotorConfig MotorRates(base::MotorConfig, base::Vendor);
base::Gains JointGains(base::Gains);
const char *SystemCheckPath();
}
