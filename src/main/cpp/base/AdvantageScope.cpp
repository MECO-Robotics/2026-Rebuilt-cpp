#include "base/AdvantageScope.h"
#include <stdexcept>
#include <WPILibVersion.h>
#include <networktables/NetworkTable.h>

namespace base {
void PublishTelemetryMetadata(std::string_view project, nt::NetworkTableInstance instance) {
  auto table = instance.GetTable("Metadata");
  table->PutString("Project",project);
  table->PutString("Language","C++");
  table->PutString("WPILib",GetWPILibVersion());
  table->PutString("Telemetry","WPILib structs / NT4 / WPILOG");
}
DriveTelemetry::DriveTelemetry(nt::NetworkTableInstance instance)
    : pose_(instance.GetStructTopic<frc::Pose2d>("/AdvantageScope/Drive/Pose").Publish()),
      robotPose_(instance.GetStructTopic<frc::Pose3d>("/AdvantageScope/Drive/RobotPose").Publish()),
      speeds_(instance.GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/Speeds").Publish()),
      measured_(instance.GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/MeasuredStates").Publish()),
      targets_(instance.GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/TargetStates").Publish()) {}
void DriveTelemetry::Publish(const frc::Pose2d& pose,const frc::ChassisSpeeds& speeds,
                            std::span<const frc::SwerveModuleState> measured,
                            std::span<const frc::SwerveModuleState> targets,std::int64_t timestampUs) {
  if((!measured.empty() && measured.size()!=4) || (!targets.empty() && targets.size()!=4))
    throw std::invalid_argument("Drive telemetry requires FL, FR, BL, BR or an empty array");
  pose_.Set(pose,timestampUs);
  robotPose_.Set(frc::Pose3d{pose},timestampUs);
  speeds_.Set(speeds,timestampUs);
  measured_.Set(measured,timestampUs);
  targets_.Set(targets,timestampUs);
}
}
