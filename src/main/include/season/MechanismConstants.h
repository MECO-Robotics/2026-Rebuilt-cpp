#pragma once
#include <array>
#include "season/Shooting.h"
namespace season::hardware
{
  // Java 2026-Rebuilt hardware values; CAN bus strings are significant.
  struct FlywheelConfig
  {
    std::array<int, 2> ids;
    std::array<bool, 2> inverted;
    int motorCount;
    double gearRatio, inertiaKgMetersSquared, currentLimitAmps;
    const char *canBus;
  };
  struct FlywheelGains
  {
    double p, i, d, s, v, a, maxAcceleration, tolerance;
  };
  inline constexpr FlywheelConfig kTopIndexer{{32, 0}, {false, false}, 1, 1, .025, 30, ""};
  inline constexpr FlywheelConfig kBottomIndexer{{31, 0}, {false, false}, 1, 1, .025, 30, "MECO 2"};
  inline constexpr FlywheelConfig kConveyor{{23, 0}, {true, false}, 1, 1, .025, 30, ""};
  inline constexpr FlywheelConfig kShooter{{34, 35}, {false, true}, 2, 22.0 / 14, 24 * .45359237 * .0254 * .0254, 40, "MECO 2"};
  inline constexpr FlywheelConfig kIntake{{22, 53}, {true, true}, 2, 1.5, .025, 40, ""};
  inline constexpr FlywheelGains kIndexerGains{0, 0, 0, 0, 0, 0, 0, 0};
  inline constexpr FlywheelGains kConveyorGains{0, 0, 0, 0, .065, 0, 0, 0};
  inline constexpr FlywheelGains kShooterGains{.8, 0, .01, .33, .19, .15, 100, .5};
  inline constexpr FlywheelGains kIntakeGains{0, 0, 0, 0, 0, 0, 1, .1};
  struct JointGains
  {
    double p, i, d, s, g, v, a, maxVelocity, maxAcceleration, minPosition, maxPosition, tolerance, gravityOffset;
  };
  inline constexpr JointGains kHoodGains{20, 0, 0, .5, .1, 0, 0, 4, 100, 0, .049, .002, 0};
  inline constexpr JointGains kRackGains{175, 0, 0, .4, .03, 0, 0, .5, 100, 0, .35, .02, 0};
  inline constexpr int kHoodCanId = 33, kRackCanId = 21;
  inline constexpr double kHoodGearRatio = 105;
  inline constexpr double kRackGearRatio = 9 / (kPi * .0254);
  inline constexpr double kRackDrumRadius = .5 * .0254;
  inline constexpr double kRackInputInertia = 15 * .45359237 * kRackDrumRadius * kRackDrumRadius / (kRackGearRatio * kRackGearRatio);
  inline constexpr double kRackStow = 0, kRackDeploy = .30, kRackSafe = .12;
  inline constexpr double kIntakeVolts = -10, kIntakeSlowVolts = 7, kEjectVolts = 10;
  inline constexpr double kConveyorFeedVolts = -11, kBottomFeedVolts = -10, kTopFeedVolts = 10;
  inline constexpr double kHubHood = 0, kFerryHood = .049, kTrenchHood = .049;
  inline constexpr double kHubRps = 30, kFerryRps = 55, kTrenchRps = 35;
  inline constexpr double kAutoReadyTimeoutSeconds = 2, kAutoFeedSeconds = 1, kAutoTotalTimeoutSeconds = 3;
}
