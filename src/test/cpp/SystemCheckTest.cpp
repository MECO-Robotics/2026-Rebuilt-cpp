#include "season/SystemCheck.h"
#include <gtest/gtest.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HAL.h>

namespace {
void ConfigureDS(bool enabled) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetFmsAttached(false);
  frc::sim::DriverStationSim::SetEStop(false);
  frc::sim::DriverStationSim::SetTest(true);
  frc::sim::DriverStationSim::SetEnabled(enabled);
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12});
  frc::sim::DriverStationSim::NotifyNewData();
}
}
TEST(SystemCheckSafety, RefusesDisabledAndDoesNotRunActions) {
  ConfigureDS(false);
  int actions=0,stops=0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  season::SystemCheck check([&]{++stops;},"/tmp/frc-systemcheck-disabled.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock",1,[&]{++actions;},[]{return season::CheckEvaluation{true,"ok",{}};}});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed",true);
  EXPECT_FALSE(check.Start());
  EXPECT_EQ(actions,0);
}
TEST(SystemCheckSafety, OwnershipInterruptionCancelsDuringCountdown) {
  ConfigureDS(true);
  int actions=0,stops=0;
  struct MockSubsystem : frc2::SubsystemBase {} mechanism;
  season::SystemCheck check([&]{++stops;},"/tmp/frc-systemcheck-interrupted.json");
  check.ConfigureRequirements({&mechanism});
  check.AddStage({"mock",1,[&]{++actions;},[]{return season::CheckEvaluation{true,"ok",{}};}});
  frc::SmartDashboard::PutBoolean("SystemCheck/Armed",true);
  frc::SmartDashboard::PutBoolean("SystemCheck/SetupConfirmed",true);
  ASSERT_TRUE(check.Start());
  EXPECT_TRUE(check.Active());
  EXPECT_EQ(actions,0);
  auto conflicting=frc2::cmd::Run([]{}, {&mechanism});
  conflicting.Schedule();
  EXPECT_FALSE(check.Active());
  EXPECT_EQ(actions,0);
  EXPECT_GE(stops,2);
  EXPECT_NE(check.ReportJson().find("ABORTED"),std::string::npos);
  conflicting.Cancel();
  ConfigureDS(false);
}
