#pragma once
#include "season/AutoDrive.h"
#include "season/Mechanisms.h"

namespace season {
// Runs until interrupted. Possession is not sensed on this robot; collection
// and shooting are bounded attempts rather than exact capacity-based decisions.
frc2::CommandPtr OffensiveAutoDrive(AutoDrive& perception, base::Drive& drive,
                                    Mechanisms mechanisms, FieldGeometry field);
}
