#include <gtest/gtest.h>
#include "base/QuestNav.h"
using namespace units::literals;
TEST(QuestNav, SuppressesUninitializedAndAlignsMountingOffset) {
  base::QuestNavConfig config;
  config.robotToHeadset = frc::Transform3d{frc::Translation3d{0.5_m, 0_m, 0_m}, frc::Rotation3d{}};
  base::QuestNav quest{config};
  EXPECT_FALSE(quest.Update({frc::Pose3d{1.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1_s, true}, 1_s));
  auto observation = quest.Update({frc::Pose3d{1.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1.02_s, true},
      1.02_s, base::QuestNavObservation{frc::Pose2d{4_m, 3_m, frc::Rotation2d{90_deg}}, 1.02_s});
  ASSERT_TRUE(observation);
  EXPECT_NEAR(observation->pose.X().value(), 4, 1e-9);
  EXPECT_NEAR(observation->pose.Y().value(), 3, 1e-9);
  observation = quest.Update({frc::Pose3d{2.5_m, 1_m, 0_m, frc::Rotation3d{}}, 1.04_s, true}, 1.04_s);
  ASSERT_TRUE(observation);
  EXPECT_NEAR(observation->pose.X().value(), 4, 1e-9);
  EXPECT_NEAR(observation->pose.Y().value(), 4, 1e-9);
  EXPECT_EQ(observation->timestamp, 1.04_s);
}
TEST(QuestNav, RejectsStaleLostAndRepeatedFramesAndRequiresFreshAlignment) {
  base::QuestNav quest;
  const frc::Pose3d raw{1_m, 1_m, 0_m, frc::Rotation3d{}};
  const base::QuestNavObservation absolute{frc::Pose2d{2_m, 2_m, frc::Rotation2d{}}, 1_s};
  EXPECT_FALSE(quest.Update({raw, 1_s, true}, 1.2_s, absolute));
  EXPECT_FALSE(quest.Update({raw, 1.2_s, false}, 1.2_s, absolute));
  EXPECT_FALSE(quest.Update({raw, 1.22_s, true}, 1.22_s, absolute));
  EXPECT_FALSE(quest.Initialized());
  ASSERT_TRUE(quest.ResetPose(absolute.pose, 1.22_s));
  EXPECT_FALSE(quest.Update({raw, 1.22_s, true}, 1.22_s));
  EXPECT_TRUE(quest.Update({raw, 1.24_s, true}, 1.24_s));
  EXPECT_FALSE(quest.Connected(1.4_s));
  quest.InvalidateAlignment();
  EXPECT_FALSE(quest.Update({raw, 1.26_s, true}, 1.26_s));
}

#include <frc/geometry/proto/Pose3dProto.h>
#include <vector>
namespace {
std::vector<std::uint8_t> FrameBytes() {
  // Literal QuestNav envelope: frame_count=42, timestamp=12.5 (little-endian
  // double), tracking=true. Pose uses WPILib's exact 2026 protobuf encoding.
  std::vector<std::uint8_t> bytes{0x08, 0x2a, 0x11, 0, 0, 0, 0, 0, 0, 0x29, 0x40, 0x20, 1};
  wpi::ProtobufMessage<frc::Pose3d> codec;
  std::vector<std::uint8_t> pose;
  codec.Pack(pose, frc::Pose3d{1_m, 2_m, 0_m, frc::Rotation3d{}});
  bytes.push_back(0x1a);
  bytes.push_back(static_cast<std::uint8_t>(pose.size()));
  bytes.insert(bytes.end(), pose.begin(), pose.end());
  return bytes;
}
}
TEST(QuestNavTransport, DecodesPinnedSchemaAndRejectsMalformedMessages) {
  auto bytes = FrameBytes();
  const auto decoded = base::DecodeQuestNavFrame(bytes);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(decoded->frameCount, 42);
  EXPECT_DOUBLE_EQ(decoded->appTimestamp, 12.5);
  EXPECT_TRUE(decoded->tracking);
  EXPECT_DOUBLE_EQ(decoded->headsetPose.X().value(), 1);
  EXPECT_DOUBLE_EQ(decoded->headsetPose.Y().value(), 2);
  // Forward-compatible unknown varint field 5.
  bytes.insert(bytes.end(), {0x28, 0x01});
  EXPECT_TRUE(base::DecodeQuestNavFrame(bytes));
  bytes.pop_back();
  EXPECT_FALSE(base::DecodeQuestNavFrame(bytes));
  EXPECT_FALSE(base::DecodeQuestNavFrame(std::array<std::uint8_t, 2>{0x20, 1}));
  EXPECT_FALSE(base::DecodeQuestNavFrame(std::vector<std::uint8_t>(2049)));
  const auto device = base::DecodeQuestNavDevice(std::array<std::uint8_t, 4>{8, 3, 24, 85});
  ASSERT_TRUE(device);
  EXPECT_EQ(device->batteryPercent, 85);
  EXPECT_EQ(device->trackingLostCounter, 3);
  EXPECT_FALSE(base::DecodeQuestNavDevice(std::array<std::uint8_t, 2>{24, 101}));
}
TEST(QuestNav, RestartAndStaleResetRequireRelocalization) {
  base::QuestNav quest;
  const frc::Pose3d raw{1_m, 1_m, 0_m, frc::Rotation3d{}};
  ASSERT_TRUE(quest.Update({raw, 1_s, true}, 1_s,
      base::QuestNavObservation{frc::Pose2d{2_m, 2_m, frc::Rotation2d{}}, 1_s}));
  EXPECT_FALSE(quest.ResetPose(frc::Pose2d{}, 1.2_s));
  EXPECT_FALSE(quest.Update({raw, 1.02_s, true, true}, 1.02_s));
  EXPECT_FALSE(quest.Initialized());
}
TEST(QuestNav, ReplacingAlignmentYawDoesNotAverageIncompatibleTranslations) {
  base::QuestNav quest;
  const frc::Pose3d raw{1_m, 1_m, 0_m, frc::Rotation3d{}};
  ASSERT_TRUE(quest.Update({raw, 1_s, true}, 1_s,
      base::QuestNavObservation{frc::Pose2d{4_m, 3_m, frc::Rotation2d{}}, 1_s}));
  const auto updated = quest.Update({raw, 1.02_s, true}, 1.02_s,
      base::QuestNavObservation{frc::Pose2d{4_m, 3_m, frc::Rotation2d{90_deg}}, 1.02_s});
  ASSERT_TRUE(updated);
  EXPECT_NEAR(updated->pose.X().value(), 4, 1e-9);
  EXPECT_NEAR(updated->pose.Y().value(), 3, 1e-9);
}
TEST(QuestNavTransport, NativeNtInjectionKeepsCaptureTimeAndDetectsRestart) {
  auto instance = nt::NetworkTableInstance::Create();
  instance.StartLocal();
  {
    base::QuestNavTransport transport{instance};
    auto publisher = instance.GetRawTopic("/QuestNav/frameData").Publish(
        "proto:questnav.protos.data.ProtobufQuestNavFrameData");
    auto bytes = FrameBytes();
    publisher.Set(bytes, 1000000);
    unsigned received = 0;
    transport.Poll([&](const base::QuestNavFrame& frame) {
      ++received;
      EXPECT_EQ(frame.timestamp, 1_s);
      EXPECT_FALSE(frame.restarted);
    });
    EXPECT_EQ(received, 1u);
    // Simulate app restart: frame count falls from 42 to 1. App uptime is
    // diagnostic only; NT capture timestamp remains monotonic in robot time.
    bytes[1] = 1;
    publisher.Set(bytes, 1020000);
    transport.Poll([&](const base::QuestNavFrame& frame) {
      ++received;
      EXPECT_EQ(frame.timestamp, 1.02_s);
      EXPECT_TRUE(frame.restarted);
    });
    EXPECT_EQ(received, 2u);
  }
  instance.StopLocal();
  nt::NetworkTableInstance::Destroy(instance);
}
TEST(QuestNav, RejectedFieldPoseCannotRemainReadyDespiteLiveTracking) {
  base::QuestNav quest;
  const frc::Pose3d raw{1_m, 1_m, 0_m, frc::Rotation3d{}};
  ASSERT_TRUE(quest.Update({raw, 1_s, true}, 1_s,
      base::QuestNavObservation{frc::Pose2d{2_m, 2_m, frc::Rotation2d{}}, 1_s}));
  ASSERT_TRUE(quest.Ready(1_s));
  EXPECT_FALSE(quest.Update({frc::Pose3d{30_m, 1_m, 0_m, frc::Rotation3d{}}, 1.02_s, true}, 1.02_s));
  EXPECT_TRUE(quest.Connected(1.02_s));
  EXPECT_FALSE(quest.Ready(1.02_s));
  // Explicit reset can recover a bad field transform while raw tracking works.
  ASSERT_TRUE(quest.ResetPose(frc::Pose2d{2_m, 2_m, frc::Rotation2d{}}, 1.02_s));
  EXPECT_TRUE(quest.Ready(1.02_s));
  quest.InvalidateAlignment();
  EXPECT_FALSE(quest.Ready(1.02_s));
}
