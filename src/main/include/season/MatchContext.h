#pragma once
#include "season/Shooting.h"
#include <optional>
#include <string>
namespace season
{
  struct MatchContext
  {
    bool enabled{}, autonomous{}, teleop{}, gameDataValid{};
    std::optional<Alliance> alliance, inactiveFirstAlliance, firstActiveAlliance;
    std::optional<double> periodTime;
    std::string gameData, phase{"unknown"};
    std::optional<bool> blueActive, redActive, ownActive;
    std::optional<double> blueNextChange, redNextChange, ownNextChange;
  };
  MatchContext ComputeMatchContext(bool enabled, bool autonomous, bool teleop,
                                   std::optional<Alliance> alliance, double time, const std::string &data);
  std::string AllianceLabel(std::optional<Alliance> alliance);
  MatchContext ReadMatchContext();
}
