#include "season/OffensiveAutoDrive.h"
#include <gtest/gtest.h>
using namespace units::literals;

TEST(OffensiveAutoDrive, HubShotRequiresKnownActiveHubAndSettledArrival) {
  const frc::Pose2d target{3_m,1_m,frc::Rotation2d{0_rad}};
  EXPECT_FALSE(season::OffensiveHubShotAllowed(std::nullopt,target,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(false,target,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,std::nullopt,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,target,{4_m,1_m,frc::Rotation2d{0_rad}},{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,target,target,{.5_mps,0_mps,0_rad_per_s}));
  EXPECT_TRUE(season::OffensiveHubShotAllowed(true,target,target,{}));
}
