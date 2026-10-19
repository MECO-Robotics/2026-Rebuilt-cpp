#include "base/QuestNav.h"
#include <cmath>
#include <limits>
#include <pb_decode.h>
#include <frc/geometry/proto/Pose3dProto.h>

// QuestNav 2026-2.2.0 schema authority: the pinned vendor sources JAR,
// gg/questnav/questnav/protos/generated/Data.java and QuestNav.java.
// frame: int32 frame_count=1, double timestamp=2, WPILib Pose3d pose3d=3,
// bool isTracking=4. Device: int32 tracking_lost_counter=1, battery_percent=3.
namespace base {
namespace {
constexpr std::size_t kMaxMessageBytes = 2048;
// The envelope fields are scalar except pose3d. Decode scalar wire fields with
// WPILib's bundled nanopb; delegate the nested WPILib pose to WPILib itself.
bool Int32(pb_istream_t& stream, std::int32_t& value) {
  std::uint64_t decoded;
  if (!pb_decode_varint(&stream, &decoded) || decoded > std::numeric_limits<std::int32_t>::max()) return false;
  value = static_cast<std::int32_t>(decoded);
  return true;
}
bool FinitePose(const frc::Pose3d& pose) {
  const auto& q = pose.Rotation().GetQuaternion();
  return std::isfinite(pose.X().value()) && std::isfinite(pose.Y().value()) &&
      std::isfinite(pose.Z().value()) && std::isfinite(q.W()) && std::isfinite(q.X()) &&
      std::isfinite(q.Y()) && std::isfinite(q.Z());
}
}
std::optional<QuestNavDecodedFrame> DecodeQuestNavFrame(std::span<const std::uint8_t> bytes) {
  if (bytes.empty() || bytes.size() > kMaxMessageBytes) return std::nullopt;
  auto stream = pb_istream_from_buffer(bytes.data(), bytes.size());
  QuestNavDecodedFrame frame;
  bool posePresent = false;
  while (stream.bytes_left) {
    pb_wire_type_t wire;
    std::uint32_t tag;
    bool eof;
    if (!pb_decode_tag(&stream, &wire, &tag, &eof) || tag == 0) return std::nullopt;
    switch (tag) {
      case 1:
        if (wire != PB_WT_VARINT || !Int32(stream, frame.frameCount)) return std::nullopt;
        break;
      case 2:
        if (wire != PB_WT_64BIT || !pb_decode_fixed64(&stream, &frame.appTimestamp) ||
            !std::isfinite(frame.appTimestamp) || frame.appTimestamp < 0) return std::nullopt;
        break;
      case 3: {
        if (wire != PB_WT_STRING) return std::nullopt;
        pb_istream_t nested;
        if (!pb_make_string_substream(&stream, &nested)) return std::nullopt;
        wpi::ProtoInputStream<frc::Pose3d> poseStream{&nested};
        auto pose = wpi::Protobuf<frc::Pose3d>::Unpack(poseStream);
        const bool closed = pb_close_string_substream(&stream, &nested);
        if (!pose || !closed || !FinitePose(*pose)) return std::nullopt;
        frame.headsetPose = *pose;
        posePresent = true;
        break;
      }
      case 4: {
        std::uint64_t tracking;
        if (wire != PB_WT_VARINT || !pb_decode_varint(&stream, &tracking) || tracking > 1) return std::nullopt;
        frame.tracking = tracking != 0;
        break;
      }
      default:
        if (!pb_skip_field(&stream, wire)) return std::nullopt;
    }
  }
  if (!posePresent) return std::nullopt;
  return frame;
}
std::optional<QuestNavDeviceData> DecodeQuestNavDevice(std::span<const std::uint8_t> bytes) {
  if (bytes.empty() || bytes.size() > kMaxMessageBytes) return std::nullopt;
  auto stream = pb_istream_from_buffer(bytes.data(), bytes.size());
  QuestNavDeviceData data;
  while (stream.bytes_left) {
    pb_wire_type_t wire;
    std::uint32_t tag;
    bool eof;
    if (!pb_decode_tag(&stream, &wire, &tag, &eof) || tag == 0) return std::nullopt;
    if (tag == 1 || tag == 3) {
      auto& destination = tag == 1 ? data.trackingLostCounter : data.batteryPercent;
      if (wire != PB_WT_VARINT || !Int32(stream, destination)) return std::nullopt;
    } else if (!pb_skip_field(&stream, wire)) return std::nullopt;
  }
  if (data.batteryPercent > 100) return std::nullopt;
  return data;
}
QuestNavTransport::QuestNavTransport(nt::NetworkTableInstance instance)
    : frames_(instance.GetRawTopic("/QuestNav/frameData").Subscribe(
          "proto:questnav.protos.data.ProtobufQuestNavFrameData", {},
          {.pollStorage = 20, .periodic = 0.01, .sendAll = true})),
      devices_(instance.GetRawTopic("/QuestNav/deviceData").Subscribe(
          "proto:questnav.protos.data.ProtobufQuestNavDeviceData", {})),
      version_(instance.GetStringTopic("/QuestNav/version").Subscribe("unknown")) {}
void QuestNavTransport::Poll(const std::function<void(const QuestNavFrame&)>& consumer) {
  for (const auto& sample : devices_.ReadQueue()) {
    auto decoded = DecodeQuestNavDevice(sample.value);
    if (decoded) device_ = decoded;
    else ++decodeFailures_;
  }
  for (const auto& sample : frames_.ReadQueue()) {
    auto decoded = DecodeQuestNavFrame(sample.value);
    if (!decoded || sample.serverTime <= 0) { ++decodeFailures_; continue; }
    const bool restarted = lastFrameCount_ && decoded->frameCount < *lastFrameCount_;
    lastFrameCount_ = decoded->frameCount;
    if (consumer) consumer({decoded->headsetPose, units::second_t{sample.serverTime * 1e-6},
        decoded->tracking, restarted});
  }
}
}
