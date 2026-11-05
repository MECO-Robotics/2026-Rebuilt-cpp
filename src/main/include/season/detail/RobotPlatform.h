#pragma once
#include "base/Mechanism.h"
namespace season::hardware
{
base::MotorConfig MotorRates(base::MotorConfig, base::Vendor);
base::Gains JointGains(base::Gains);
const char *SystemCheckPath();
}
