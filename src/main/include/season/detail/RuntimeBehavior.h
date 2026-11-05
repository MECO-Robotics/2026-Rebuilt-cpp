#pragma once

#include <functional>

namespace season
{
  struct Mechanisms;
  namespace runtime
  {
    // Environment-dependent defaults and measured transport feedback.
    struct Behavior
    {
      double defaultCapacity = 0;
      double defaultFeedRate = 0;
      double emptyQuietSeconds = .4;
      double emptyShooterCurrentAmps = 0;
      double emptyRollerNoiseAmps = 0;
      double shotTimeoutSeconds = 15;
      std::function<bool(const Mechanisms &)> transportActive = [](const Mechanisms &) { return true; };
    };
    Behavior RuntimeBehavior();
  }
}

#ifndef __FRC_ROBORIO__
namespace season::simulation
{
  // Voltage alone cannot establish direction during a roller reversal.
  bool TransportFeeding(const Mechanisms &mechanisms);
}
#endif
