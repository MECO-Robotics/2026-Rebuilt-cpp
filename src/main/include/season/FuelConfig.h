#pragma once

namespace season::fuelconfig
{
// Desktop fuel-model configuration. These values also define the nominal
// rate used by offensive feed timing when no simulation preference is set.
inline constexpr int kSimulationCapacity = 54;
inline constexpr int kSimulationBurstSize = 4;
inline constexpr double kSimulationBurstPeriodSeconds = .20;
inline constexpr double kSimulationFeedBallsPerSecond =
    kSimulationBurstSize / kSimulationBurstPeriodSeconds;
}
