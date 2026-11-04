#pragma once
#include "season/shooting/Shooting.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
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
 frc2::Command *Selected() { return chooser_.GetSelected(); }
private:
 season::ShotTarget SelectTarget();
 frc2::CommandPtr HubShot(bool ownDrive = true, bool requireVision = false);
 RobotContainer &owner_;
 frc::SendableChooser<frc2::Command *> chooser_;
 std::vector<frc2::CommandPtr> autos_;
};
}
