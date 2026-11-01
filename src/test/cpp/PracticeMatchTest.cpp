#ifndef __FRC_ROBORIO__
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <hal/HAL.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <meco/sim/PracticeMatchController.h>
namespace {
using DS=frc::sim::DriverStationSim;
class PracticeMatchTest:public testing::Test {
 protected:
  nt::NetworkTableInstance nt;
  std::shared_ptr<nt::NetworkTable> table;
  void SetUp() override {
    HAL_Initialize(500,0);frc::sim::PauseTiming();DS::ResetData();
    nt=nt::NetworkTableInstance::Create();nt.StartLocal();table=nt.GetTable("/Sim/Practice");
  }
  void TearDown() override {nt.StopLocal();nt::NetworkTableInstance::Destroy(nt);DS::ResetData();DS::NotifyNewData();frc::sim::ResumeTiming();}
  void Tick(meco::sim::PracticeMatchController& controller,double seconds) {
    frc::sim::StepTiming(units::second_t{seconds});controller.Update();frc::DriverStation::RefreshData();
  }
};
TEST_F(PracticeMatchTest, ExternalInterruptionAndInvalidConfigurationNeverReenable) {
  meco::sim::PracticeMatchController controller({20,3,140},nt);
  auto command=table->GetStringTopic("Command").Publish({.keepDuplicates=true});
  table->GetEntry("Config/AutoResult").SetString("invalid");command.Set("start");Tick(controller,0);EXPECT_FALSE(DS::GetEnabled());
  table->GetEntry("Config/AutoResult").SetString("B");
  table->GetEntry("Config/AllianceStation").SetString("Red1");
  for(int kind=0;kind<3;++kind) {
    command.Set("start");Tick(controller,0);ASSERT_TRUE(DS::GetEnabled());
    EXPECT_EQ(DS::GetAllianceStationId(),HAL_AllianceStationID_kRed1);
    if(kind==0) {DS::SetEnabled(false);DS::SetEnabled(true);} // brief disable must be latched
    if(kind==1) DS::SetTest(true);
    if(kind==2) DS::SetEStop(true);
    Tick(controller,0);EXPECT_FALSE(DS::GetEnabled());EXPECT_EQ(table->GetEntry("State").GetString(""),"aborted");
    Tick(controller,200);EXPECT_FALSE(DS::GetEnabled());
  }
  command.Set("start");Tick(controller,0);EXPECT_FALSE(DS::GetEnabled());EXPECT_TRUE(DS::GetEStop());
}

}
#endif
