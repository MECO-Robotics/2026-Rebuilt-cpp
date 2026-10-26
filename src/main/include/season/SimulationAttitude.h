#pragma once

#ifndef __FRC_ROBORIO__
#include <frc/geometry/Rotation3d.h>

namespace season::simulation {
// Written by external-plant feedback and read by RobotContainer on the same
// simulation owner thread. The vendordep's DriveSimulationTruth omits tilt.
inline frc::Rotation3d chassisAttitude;
}
#endif
