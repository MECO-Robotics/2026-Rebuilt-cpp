#include "season/RobotVisualization.h"
#include <gtest/gtest.h>

TEST(RemyVisualization, DeploymentAndPivotRotationsMatchAssetFrame) {
  const auto zero = season::RemyComponentPoses(0, 0, 0);
  for (const auto& pose : zero) EXPECT_NEAR(pose.Translation().Norm().value(), 0, 1e-12);
  const auto poses = season::RemyComponentPoses(.30, .25, .25);
  EXPECT_GT(poses[2].X().value(), .29);
  EXPECT_LT(poses[2].Z().value(), -.039);
  EXPECT_NEAR(poses[3].X().value(), poses[2].X().value(), 1e-12);
  EXPECT_NEAR(poses[4].Z().value(), 0, 1e-12);
  const frc::Translation3d pivot{units::meter_t{-.103310}, units::meter_t{0}, units::meter_t{.423}};
  // A quarter turn moves the component origin but leaves its axle fixed.
  for (int i : {0, 1}) {
    const auto transformed = poses[i].Translation() + pivot.RotateBy(poses[i].Rotation());
    EXPECT_NEAR((transformed - pivot).Norm().value(), 0, 1e-12);
    EXPECT_NEAR(poses[i].Rotation().Y().value(), std::numbers::pi/2, 1e-12);
  }
}
