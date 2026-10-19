#pragma once
#include "base/Drive.h"
#include "base/Vision.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
 private:
  base::Drive drive_;
  frc2::CommandXboxController controller_{0};
  frc::SendableChooser<frc2::Command*> autoChooser_;
};
