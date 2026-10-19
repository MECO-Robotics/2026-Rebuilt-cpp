#pragma once
#include <cstdint>
#include <span>
#include <string_view>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

namespace base {
// Standard string metadata recognized in both NT4 and WPILib NT log captures.
void PublishTelemetryMetadata(std::string_view project,
    nt::NetworkTableInstance instance=nt::NetworkTableInstance::GetDefault());
// Standard WPILib structs are discoverable in AdvantageScope via NT4 and WPILOG.
// Angles are radians, translation/speed metres and metres/s. Module array order
// is front-left, front-right, back-left, back-right, supplied by the caller.
class DriveTelemetry {
 public:
  explicit DriveTelemetry(nt::NetworkTableInstance instance=nt::NetworkTableInstance::GetDefault());
  // One shared FPGA timestamp per snapshot. Arrays contain four modules, or zero
  // elements to explicitly clear unavailable data. Call on the main telemetry loop.
  void Publish(const frc::Pose2d& pose,const frc::ChassisSpeeds& speeds,
               std::span<const frc::SwerveModuleState> measured,
               std::span<const frc::SwerveModuleState> targets,std::int64_t timestampUs);
 private:
  nt::StructPublisher<frc::Pose2d> pose_;
  nt::StructPublisher<frc::Pose3d> robotPose_;
  nt::StructPublisher<frc::ChassisSpeeds> speeds_;
  nt::StructArrayPublisher<frc::SwerveModuleState> measured_,targets_;
};
}
