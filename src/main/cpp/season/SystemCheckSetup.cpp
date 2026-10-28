#include "season/SystemCheckSetup.h"
#include <cmath>
#include <memory>
#include <frc/RobotController.h>

namespace season {
void ConfigureSystemChecks(base::SystemCheck& check,base::Drive& drive,
                           base::PositionJoint& rack,base::PositionJoint& hood,
                           base::Flywheel& shooter,
                           std::vector<std::pair<std::string,base::Flywheel*>> rollers,
                           std::function<bool()> visionConnected) {
  auto rackStart=std::make_shared<double>(0);
  auto rackStarted=std::make_shared<bool>(false);
  check.AddStage({"preflight",3,[rackStarted]{*rackStarted=false;},[&rack,&hood,&shooter,rollers]{
    bool connected=rack.Inputs().connected && hood.Inputs().connected && shooter.Inputs().connected;
    for(auto& [name,roller]:rollers) connected=connected && roller && roller->Inputs().connected;
    return base::CheckEvaluation{connected,"Required mechanism controller connectivity",
                          {{"batteryVolts",frc::RobotController::GetBatteryVoltage().value()}}};
  },{},.25});
  check.AddStage({"swerve-forward",4,[&drive]{
    drive.RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{.5},units::meters_per_second_t{0},units::radians_per_second_t{0}});
  },[&drive]{auto speeds=drive.Speeds();return base::CheckEvaluation{
    std::abs(speeds.vx.value()-.5)<.15 && std::abs(speeds.vy.value())<.15,
    "Unloaded forward velocity tracking",{{"driveVx",speeds.vx.value()},{"driveVy",speeds.vy.value()}}};
  },{"preflight"},.5});
  // A relative 2 cm rack movement avoids assuming an uncalibrated absolute home reference.
  check.AddStage({"rack-motion",5,[&rack,rackStart,rackStarted]{
    if(!*rackStarted){*rackStart=rack.Position();*rackStarted=true;}
    rack.SetPosition(*rackStart+.02);
  },[&rack,rackStart]{return base::CheckEvaluation{
    rack.Inputs().connected && std::abs(rack.Position()-(*rackStart+.02))<.01,
    "Rack commanded 2 cm from initial calibrated position",
    {{"rackPosition",rack.Position()},{"rackVelocity",rack.Velocity()}}};
  },{"preflight"},.5});
  check.AddStage({"hood-position",4,[&hood]{hood.SetPosition(.020);},[&hood]{return base::CheckEvaluation{
    hood.Inputs().connected && std::abs(hood.Position()-.020)<.003,
    "Hood target .020 rotations",{{"hoodPosition",hood.Position()},{"hoodVelocity",hood.Velocity()}}};
  },{"preflight"},.5});
  for(auto& [name,roller]:rollers) {
    if(!roller) throw std::invalid_argument("Null diagnostic roller");
    check.AddStage({name+"-roller",3,[roller]{roller->SetVoltage(2);},[roller]{return base::CheckEvaluation{
      roller->Inputs().connected && roller->Velocity()>1,
      "Positive roller motion at 2 V",{{"rollerVelocity",roller->Velocity()},{"rollerCurrent",roller->Inputs().current}}};
    },{"preflight"},.5});
  }
  check.AddStage({"shooter-speed",5,[&shooter]{shooter.SetVelocity(10);},[&shooter]{return base::CheckEvaluation{
    shooter.Inputs().connected && std::abs(shooter.Velocity()-10)<2,
    "Unloaded shooter target 10 rps",{{"shooterVelocity",shooter.Velocity()},{"shooterCurrent",shooter.Inputs().current}}};
  },{"preflight"},.5});
  check.AddStage({"vision-connectivity",5,[]{},[visionConnected]{bool connected=visionConnected && visionConnected();
    return base::CheckEvaluation{connected,"Camera heartbeat connectivity only; accepted localization is not verified",{{"visionConnected",connected ? 1. : 0.}}};
  },{},.5});
}
}
