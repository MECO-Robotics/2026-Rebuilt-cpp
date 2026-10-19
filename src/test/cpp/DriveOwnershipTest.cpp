#include "base/Drive.h"
#include <gtest/gtest.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/HAL.h>

TEST(DriveOwnership, AimPreemptsJoystickAndDisableCancelsOwner) {
  HAL_Initialize(500, 0);
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.CancelAll();
  scheduler.Enable();
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetAutonomous(false);
  frc::sim::DriverStationSim::SetTest(false);
  frc::sim::DriverStationSim::SetEStop(false);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();

  base::Drive drive;
  auto joystick = drive.Joystick([] { return .5; }, [] { return 0.; }, [] { return 0.; });
  auto aim = drive.Aim([] { return 0.; }, [] { return 0.; }, [] { return frc::Rotation2d{}; });
  joystick.Schedule();
  scheduler.Run();
  EXPECT_EQ(scheduler.Requiring(&drive), joystick.get());
  EXPECT_TRUE(joystick.IsScheduled());

  aim.Schedule();
  scheduler.Run();
  EXPECT_FALSE(joystick.IsScheduled());
  EXPECT_TRUE(aim.IsScheduled());
  EXPECT_EQ(scheduler.Requiring(&drive), aim.get());

  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();
  scheduler.Run();
  EXPECT_FALSE(aim.IsScheduled());
  EXPECT_EQ(scheduler.Requiring(&drive), nullptr);
  // Motion/neutral-output timing belongs to the running native simulator test;
  // this regression checks real command ownership and cancellation only.
  scheduler.CancelAll();
}
