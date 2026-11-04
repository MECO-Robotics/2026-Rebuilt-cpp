#pragma once

#include "base/Drive.h"

namespace season::simulation
{
#ifndef __FRC_ROBORIO__
  // Apply the desktop plant and controller assumptions before constructing Drive.
  base::DriveConfig ConfigureDrive(base::DriveConfig config);
#endif
}
