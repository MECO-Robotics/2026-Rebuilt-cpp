#include "base/QuestNav.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace base {
namespace {
bool Finite(const frc::Pose3d& pose) {
  return std::isfinite(pose.X().value()) && std::isfinite(pose.Y().value()) &&
      std::isfinite(pose.Z().value()) && std::isfinite(pose.Rotation().X().value()) &&
      std::isfinite(pose.Rotation().Y().value()) && std::isfinite(pose.Rotation().Z().value());
}
}
QuestNav::QuestNav(QuestNavConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.maxAge.value()) || config_.maxAge <= units::second_t{0} ||
      !std::isfinite(config_.maxAlignmentSkew.value()) || config_.maxAlignmentSkew < units::second_t{0} ||
      !std::isfinite(config_.fieldLength) || config_.fieldLength <= 0 ||
      !std::isfinite(config_.fieldWidth) || config_.fieldWidth <= 0 ||
      !std::isfinite(config_.maxZ) || config_.maxZ < 0 ||
      !Finite(frc::Pose3d{}.TransformBy(config_.robotToHeadset))) {
    throw std::invalid_argument("Invalid QuestNav geometry or timing configuration");
  }
}
bool QuestNav::ValidFieldPose(const frc::Pose3d& pose) const {
  return Finite(pose) && pose.X().value() >= 0 && pose.X().value() <= config_.fieldLength &&
      pose.Y().value() >= 0 && pose.Y().value() <= config_.fieldWidth &&
      std::abs(pose.Z().value()) <= config_.maxZ;
}
bool QuestNav::Connected(units::second_t now) const {
  return tracking_ && std::isfinite(now.value()) && lastTimestamp_ >= units::second_t{0} &&
      now >= lastTimestamp_ && now - lastTimestamp_ < config_.maxAge;
}
void QuestNav::InvalidateAlignment() {
  aligned_ = false;
  count_ = index_ = 0;
  offset_ = frc::Translation3d{};
  yawOffset_ = frc::Rotation3d{};
}
std::optional<QuestNavObservation> QuestNav::Update(
    const QuestNavFrame& frame, units::second_t now,
    std::optional<QuestNavObservation> absolute) {
  if (frame.restarted) InvalidateAlignment();
  tracking_ = frame.tracking;
  if (!frame.tracking || !Finite(frame.headsetPose) || !std::isfinite(now.value()) ||
      !std::isfinite(frame.timestamp.value()) || frame.timestamp < units::second_t{0} ||
      frame.timestamp <= lastTimestamp_ || frame.timestamp > now ||
      now - frame.timestamp >= config_.maxAge) {
    tracking_ = false;
    ++rejected_;
    return std::nullopt;
  }
  lastTimestamp_ = frame.timestamp;
  lastRaw_ = frame.headsetPose.TransformBy(config_.robotToHeadset.Inverse());
  if (absolute && std::isfinite(absolute->timestamp.value()) &&
      absolute->timestamp > lastAbsoluteTimestamp_ && absolute->timestamp <= now &&
      units::math::abs(absolute->timestamp - frame.timestamp) <= config_.maxAlignmentSkew &&
      ValidFieldPose(frc::Pose3d{absolute->pose})) {
    const auto previousYaw = yawOffset_;
    yawOffset_ = frc::Rotation3d{units::radian_t{0}, units::radian_t{0},
        absolute->pose.Rotation().Radians() - lastRaw_.Rotation().Z()};
    // Existing offsets use the previous yaw frame. A changed heading requires
    // recomputing offsets, rather than averaging incompatible transforms.
    if (std::abs((yawOffset_ - previousYaw).Z().value()) > 1e-9) count_ = index_ = 0;
    offsets_[index_] = frc::Pose3d{absolute->pose}.Translation() -
        lastRaw_.Translation().RotateBy(yawOffset_);
    index_ = (index_ + 1) % offsets_.size();
    count_ = std::min(count_ + 1, offsets_.size());
    offset_ = frc::Translation3d{};
    for (std::size_t i = 0; i < count_; ++i) offset_ = offset_ + offsets_[i];
    offset_ = offset_ / static_cast<double>(count_);
    lastAbsoluteTimestamp_ = absolute->timestamp;
    aligned_ = true;
  }
  if (!aligned_) return std::nullopt;
  const frc::Pose3d pose{lastRaw_.Translation().RotateBy(yawOffset_) + offset_,
      lastRaw_.Rotation() + yawOffset_};
  if (!ValidFieldPose(pose)) { ++rejected_; return std::nullopt; }
  return QuestNavObservation{pose.ToPose2d(), frame.timestamp};
}
bool QuestNav::ResetPose(const frc::Pose2d& fieldPose, units::second_t now) {
  if (!Connected(now) || !ValidFieldPose(frc::Pose3d{fieldPose})) return false;
  InvalidateAlignment();
  yawOffset_ = frc::Rotation3d{units::radian_t{0}, units::radian_t{0},
      fieldPose.Rotation().Radians() - lastRaw_.Rotation().Z()};
  offset_ = frc::Pose3d{fieldPose}.Translation() - lastRaw_.Translation().RotateBy(yawOffset_);
  aligned_ = true;
  return true;
}
}
