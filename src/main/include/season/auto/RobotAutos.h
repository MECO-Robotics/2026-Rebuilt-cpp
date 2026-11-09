#pragma once
#include "season/shooting/Shooting.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <string>
#include <unordered_map>
#include <vector>
class RobotContainer;
namespace base { class Drive; }
namespace season::autonomous {
// Settings for the named spin-up action and the Choreo chooser option.
namespace config {
inline constexpr double kAutoSpinUpTimeoutSeconds = 2.;
inline constexpr double kChoreoTranslationP = 10.;
inline constexpr double kChoreoRotationP = 7.;
}
class RobotAutos {
public:
 explicit RobotAutos(RobotContainer &owner) : owner_(owner) {}
 static void ConfigureDrive(base::Drive &);
 void Configure();
 frc2::Command *Selected();
private:
 void AddChooserOption(std::string label, std::string id, frc2::CommandPtr command, bool isDefault = false);
 season::ShotTarget SelectTarget();
 frc2::CommandPtr HubShot(bool ownDrive = true, bool requireVision = false);
 RobotContainer &owner_;
 frc::SendableChooser<std::string> chooser_;
 std::unordered_map<std::string, std::string> idsByLabel_;
 std::unordered_map<std::string, frc2::Command *> commandsById_;
 std::vector<frc2::CommandPtr> autos_;
};
}
