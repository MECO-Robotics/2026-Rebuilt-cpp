#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/SimulationDiagnostics.h"
namespace season::simulation
{
    using meco::sim::BeginLatencyProbe;
    using meco::sim::EndLatencyProbe;
    using meco::sim::LatencyProbeEnabled;
    using meco::sim::ObserveJoystick;
}
#else
namespace season::simulation
{
    inline double ObserveJoystick(double value) { return value; }
    inline void BeginLatencyProbe() {}
    inline void EndLatencyProbe() {}
}
#endif
