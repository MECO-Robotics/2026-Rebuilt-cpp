#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/ParticleContacts.h"

namespace season::simulation
{
  inline meco::sim::RobotContactConfig RobotContacts()
  {
    constexpr double inchesToMetres = .0254;
    meco::sim::RobotContactConfig c;
    // Unmeasured construction/material estimates: replace with loaded measurements.
    c.chassisClearance = 2.0 * inchesToMetres;
    c.panThickness = .125 * inchesToMetres;
    c.panHalfLength = c.panHalfWidth = .43 - 3.5 * inchesToMetres;
    c.bumperFloorGap = .5 * inchesToMetres; // User-confirmed flush foam/plywood floor gap.
    c.noodleDiameter = 2.75 * inchesToMetres;
    c.backingThickness = .75 * inchesToMetres;
    c.backingHeight = 5.0 * inchesToMetres;
    c.backingFloorGap = c.bumperFloorGap; // Selected flush plywood bottom is rigid.
    c.compressionStrain = {0, .25, .5, .7, .8};
    c.compressionStressPa = {0, 10000, 35000, 100000, 250000};
    // Overdamped foam dissipates more impact energy and reduces rebound.
    c.dampingRatio = 1.2;
    return c;
  }
}
#endif
