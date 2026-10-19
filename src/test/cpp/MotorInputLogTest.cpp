#include "base/MotorInputLog.h"
#include <gtest/gtest.h>
#include <wpi/DataLogWriter.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/raw_ostream.h>

TEST(MotorInputLog, NativeLogRoundTripPreservesTimestampAndAllInputsWithoutHardware) {
  std::string bytes;
  {
    wpi::log::DataLogWriter writer(std::make_unique<wpi::raw_string_ostream>(bytes));
    wpi::log::DoubleArrayLogEntry entry(writer,"Wheel/MotorInputs",base::kMotorInputSchema);
    entry.Append(base::EncodeMotorInputs({1,2,3,11,5,true}),4000);
    entry.Append(base::EncodeMotorInputs({6,7,8,10,9,false}),8000);
    writer.Flush();
  }
  auto buffer=wpi::MemoryBuffer::GetMemBufferCopy(std::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(bytes.data()),bytes.size()));
  wpi::log::DataLogReader reader(std::move(buffer));
  auto samples=base::ReadMotorInputLog(reader,"Wheel/MotorInputs");
  ASSERT_EQ(samples.size(),2u);
  EXPECT_EQ(samples[0].timestampMicroseconds,4000);
  base::ReplayMotorIO replay(samples);
  replay.AdvanceTo(3999);
  EXPECT_FALSE(replay.Read().connected);
  replay.AdvanceTo(4000);
  EXPECT_EQ(base::EncodeMotorInputs(replay.Read()),(std::array<double,6>{1,2,3,11,5,1}));
  replay.AdvanceTo(8000);
  EXPECT_EQ(base::EncodeMotorInputs(replay.Read()),(std::array<double,6>{6,7,8,10,9,0}));
}
