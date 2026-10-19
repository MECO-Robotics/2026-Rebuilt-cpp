#pragma once

#include <array>
#include <optional>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/time.h>

namespace base {
// Timestamp is the synchronized NT server timestamp, not Quest app uptime.
struct QuestNavFrame {
  frc::Pose3d headsetPose;
  units::second_t timestamp{0};
  bool tracking = false;
  bool restarted = false;
};
struct QuestNavObservation {
  frc::Pose2d pose;
  units::second_t timestamp{0};
};
struct QuestNavConfig {
  frc::Transform3d robotToHeadset;
  units::second_t maxAge{0.12};
  units::second_t maxAlignmentSkew{0.03};
  double fieldLength = 17.548;
  double fieldWidth = 8.052;
  double maxZ = 0.75;
};
// Deterministic estimator-side adapter. Call on one thread; no hardware is
// constructed. Transport must supply decoded QuestNav 2026 protobuf frames.
// Unaligned or stale tracking never generates an inertial observation; callers
// retain their absolute-vision fallback. Reset uses a local field transform and
// does not asynchronously reset the headset's own coordinate system.
class QuestNav {
 public:
  explicit QuestNav(QuestNavConfig config = {});
  std::optional<QuestNavObservation> Update(
      const QuestNavFrame& frame, units::second_t now,
      std::optional<QuestNavObservation> absolute = std::nullopt);
  bool ResetPose(const frc::Pose2d& fieldPose, units::second_t now);
  void InvalidateAlignment();
  bool Initialized() const { return aligned_; }
  bool Connected(units::second_t now) const;
  units::second_t LastAbsoluteTimestamp() const { return lastAbsoluteTimestamp_; }
  unsigned RejectedFrames() const { return rejected_; }
 private:
  bool ValidFieldPose(const frc::Pose3d& pose) const;
  QuestNavConfig config_;
  std::array<frc::Translation3d, 15> offsets_{};
  frc::Translation3d offset_;
  frc::Rotation3d yawOffset_;
  frc::Pose3d lastRaw_;
  units::second_t lastTimestamp_{-1};
  units::second_t lastAbsoluteTimestamp_{-1};
  std::size_t count_ = 0;
  std::size_t index_ = 0;
  unsigned rejected_ = 0;
  bool aligned_ = false;
  bool tracking_ = false;
};
}

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <networktables/StringTopic.h>
namespace base {
struct QuestNavDecodedFrame {
  frc::Pose3d headsetPose;
  double appTimestamp = 0;
  std::int32_t frameCount = 0;
  bool tracking = false;
};
struct QuestNavDeviceData {
  std::int32_t trackingLostCounter = 0;
  std::int32_t batteryPercent = 0;
};
// Schema pinned to QuestNav 2026-2.2.0. Unknown protobuf fields are skipped;
// malformed messages, wrong wire types, missing pose, and >2 KiB are rejected.
std::optional<QuestNavDecodedFrame> DecodeQuestNavFrame(std::span<const std::uint8_t> bytes);
std::optional<QuestNavDeviceData> DecodeQuestNavDevice(std::span<const std::uint8_t> bytes);
// Read-only live transport; no actuator access or headset reset requests.
// Poll on the same thread as QuestNav. Subscriber queue holds 20 frames (~200 ms
// at 100 Hz); callers must poll at least every 20 ms to preserve normal samples.
class QuestNavTransport {
 public:
  explicit QuestNavTransport(nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault());
  void Poll(const std::function<void(const QuestNavFrame&)>& consumer);
  const std::optional<QuestNavDeviceData>& Device() const { return device_; }
  std::string Version() const { return version_.Get(); }
  unsigned DecodeFailures() const { return decodeFailures_; }
 private:
  nt::RawSubscriber frames_;
  nt::RawSubscriber devices_;
  nt::StringSubscriber version_;
  std::optional<QuestNavDeviceData> device_;
  std::optional<std::int32_t> lastFrameCount_;
  unsigned decodeFailures_ = 0;
};
}
