#include "base/AdvantageScope.h"
#include <gtest/gtest.h>
#include <array>
#include <map>
#include <set>
#include <string>
#include <wpi/DataLogReader.h>
#include <wpi/DataLogWriter.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/raw_ostream.h>

TEST(AdvantageScopeTelemetry, NativeStructSchemasUnitsOrderingAndClearsSurviveNTLogRoundTrip) {
  auto nt=nt::NetworkTableInstance::Create();
  std::string bytes;
  {
    wpi::log::DataLogWriter writer(std::make_unique<wpi::raw_string_ostream>(bytes));
    auto logger=nt.StartEntryDataLog(writer,"","");
    {
      base::DriveTelemetry telemetry(nt);
      auto pose=nt.GetStructTopic<frc::Pose2d>("/AdvantageScope/Drive/Pose").Subscribe({});
      auto robot=nt.GetStructTopic<frc::Pose3d>("/AdvantageScope/Drive/RobotPose").Subscribe({});
      auto speeds=nt.GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/Speeds").Subscribe({});
      auto measured=nt.GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/MeasuredStates").Subscribe({});
      auto targets=nt.GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/TargetStates").Subscribe({});
      const frc::Pose2d expectedPose{units::meter_t{1.25},units::meter_t{-2},frc::Rotation2d{units::radian_t{.7}}};
      const frc::ChassisSpeeds expectedSpeeds{units::meters_per_second_t{2},units::meters_per_second_t{-3},units::radians_per_second_t{.8}};
      std::array<frc::SwerveModuleState,4> states;
      for(int i=0;i<4;++i) states[i]={units::meters_per_second_t{i+.5},frc::Rotation2d{units::radian_t{.2*i}}};
      telemetry.Publish(expectedPose,expectedSpeeds,states,states,10000);
      ASSERT_TRUE(nt.WaitForListenerQueue(1));
      EXPECT_EQ(pose.GetTopic().GetTypeString(),"struct:Pose2d");
      EXPECT_EQ(robot.GetTopic().GetTypeString(),"struct:Pose3d");
      EXPECT_EQ(speeds.GetTopic().GetTypeString(),"struct:ChassisSpeeds");
      EXPECT_EQ(measured.GetTopic().GetTypeString(),"struct:SwerveModuleState[]");
      EXPECT_EQ(pose.Get(),expectedPose);
      EXPECT_DOUBLE_EQ(robot.Get().Z().value(),0);
      EXPECT_NEAR(robot.Get().Rotation().Z().value(),.7,1e-12);
      EXPECT_DOUBLE_EQ(speeds.Get().omega.value(),.8);
      auto received=measured.Get();
      ASSERT_EQ(received.size(),4u);
      for(int i=0;i<4;++i) {
        EXPECT_DOUBLE_EQ(received[i].speed.value(),i+.5);
        EXPECT_NEAR(received[i].angle.Radians().value(),.2*i,1e-12);
      }
      telemetry.Publish({}, {}, {}, {},20000);
      ASSERT_TRUE(nt.WaitForListenerQueue(1));
      EXPECT_EQ(pose.Get(),frc::Pose2d{});
      EXPECT_EQ(speeds.Get().vx.value(),0);
      EXPECT_TRUE(measured.Get().empty());
      EXPECT_TRUE(targets.Get().empty());
    }
    ASSERT_TRUE(nt.WaitForListenerQueue(1));
    nt::NetworkTableInstance::StopEntryDataLog(logger);
    writer.Flush();
  }
  nt::NetworkTableInstance::Destroy(nt);
  auto buffer=wpi::MemoryBuffer::GetMemBufferCopy(std::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(bytes.data()),bytes.size()));
  wpi::log::DataLogReader reader(std::move(buffer));
  ASSERT_TRUE(reader.IsValid());
  std::map<int,std::string> names;
  std::set<std::string> schemas;
  bool sawPose=false,sawSpeeds=false,sawStates=false,sawClearedStates=false;
  for(const auto& record:reader) {
    if(record.IsStart()) {
      wpi::log::StartRecordData start;
      ASSERT_TRUE(record.GetStartData(&start));
      names[start.entry]=start.name;
      if(start.type=="structschema") schemas.insert(std::string(start.name));
    } else if(!record.IsControl()) {
      const auto& name=names[record.GetEntry()];
      if(record.GetTimestamp()==10000 && name=="/AdvantageScope/Drive/Pose") {
        auto pose=wpi::Struct<frc::Pose2d>::Unpack(record.GetRaw());
        EXPECT_DOUBLE_EQ(pose.X().value(),1.25);
        EXPECT_DOUBLE_EQ(pose.Y().value(),-2);
        EXPECT_NEAR(pose.Rotation().Radians().value(),.7,1e-12);
        sawPose=true;
      }
      if(record.GetTimestamp()==10000 && name=="/AdvantageScope/Drive/Speeds") {
        auto speeds=wpi::Struct<frc::ChassisSpeeds>::Unpack(record.GetRaw());
        EXPECT_DOUBLE_EQ(speeds.vx.value(),2);
        EXPECT_DOUBLE_EQ(speeds.vy.value(),-3);
        EXPECT_DOUBLE_EQ(speeds.omega.value(),.8);
        sawSpeeds=true;
      }
      if(name=="/AdvantageScope/Drive/MeasuredStates") {
        if(record.GetTimestamp()==10000) {
          constexpr auto size=wpi::Struct<frc::SwerveModuleState>::GetSize();
          ASSERT_EQ(record.GetRaw().size(),4*size);
          for(int i=0;i<4;++i) {
            auto state=wpi::Struct<frc::SwerveModuleState>::Unpack(record.GetRaw().subspan(i*size,size));
            EXPECT_DOUBLE_EQ(state.speed.value(),i+.5);
            EXPECT_NEAR(state.angle.Radians().value(),.2*i,1e-12);
          }
          sawStates=true;
        } else if(record.GetTimestamp()==20000) {
          EXPECT_TRUE(record.GetRaw().empty()); sawClearedStates=true;
        }
      }
    }
  }
  EXPECT_TRUE(sawPose && sawSpeeds && sawStates && sawClearedStates);
  for(const auto* schema:{"Pose2d","Pose3d","Translation2d","Rotation2d","Translation3d","Rotation3d","Quaternion","ChassisSpeeds","SwerveModuleState"})
    EXPECT_TRUE(schemas.contains(std::string("/.schema/struct:")+schema)) << schema;
}
