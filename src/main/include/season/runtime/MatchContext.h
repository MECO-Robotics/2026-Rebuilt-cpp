#pragma once

#include <base/DriverStationMatchInfo.h>
#include <optional>
#include <string>

namespace season::runtime {
struct MatchContext {
  bool enabled{}, autonomous{}, teleop{}, gameDataAvailable{}, gameDataValid{}, matchPolicyInputsKnown{true};
  std::optional<frc::DriverStation::Alliance> alliance, inactiveFirstAlliance, firstActiveAlliance;
  std::optional<double> periodTime;
  std::string gameData, phase{"unknown"};
  std::optional<bool> blueActive, redActive, ownActive;
  std::optional<double> blueNextChange, redNextChange, ownNextChange;
};

MatchContext ComputeMatchContext(const base::DriverStationMatchInfo &info);
std::string AllianceLabel(std::optional<frc::DriverStation::Alliance> alliance);
MatchContext ReadMatchContext();
inline MatchContext CurrentMatchContext() { return ReadMatchContext(); }
inline bool AutonomousEnabled() {
  const auto match = CurrentMatchContext();
  return match.enabled && match.autonomous;
}
inline bool TeleopEnabled() {
  const auto match = CurrentMatchContext();
  return match.enabled && match.teleop;
}
}  // namespace season::runtime
