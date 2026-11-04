#pragma once

namespace season
{
  struct Mechanisms;
  namespace simulation
  {
#ifndef __FRC_ROBORIO__
    // Voltage alone cannot establish direction during a roller reversal.
    bool TransportFeeding(const Mechanisms &mechanisms);
#endif
  }
}
