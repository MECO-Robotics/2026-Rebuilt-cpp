#include "season/AutoDrive.h"
#include <gtest/gtest.h>
#include <units/angle.h>

TEST(AutoDriveArrival, RequiresTargetPoseAndSettledMotion) {
  using namespace units::literals;
  const frc::Pose2d goal{3_m, 1_m, frc::Rotation2d{0_rad}};
  EXPECT_TRUE(season::AutoDriveArrived(goal, {}, goal));
  EXPECT_FALSE(season::AutoDriveArrived({3.2_m, 1_m, frc::Rotation2d{0_rad}}, {}, goal));
  EXPECT_FALSE(season::AutoDriveArrived({3_m, 1_m, frc::Rotation2d{.2_rad}}, {}, goal));
  EXPECT_TRUE(season::AutoDriveArrived(goal, {0.16_mps, 0_mps, 0_rad_per_s}, goal));
  EXPECT_FALSE(season::AutoDriveArrived(goal, {0.5_mps, 0_mps, 0_rad_per_s}, goal));
}
