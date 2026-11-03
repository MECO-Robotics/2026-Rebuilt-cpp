#pragma once
#include "base/Mechanism.h"
#include "season/mechanisms/Mechanisms.h"
namespace season::hardware {
base::Mode Mode();
bool FastControlEnabled();
base::MotorConfig MotorRates(base::MotorConfig, base::Vendor);
bool MechanismCanFd();
base::Gains JointGains(base::Gains);
const char *SystemCheckPath();
}
