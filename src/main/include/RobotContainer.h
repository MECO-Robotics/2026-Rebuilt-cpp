#pragma once
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/RobotControlConfig.h"
#include "base/SystemCheck.h"
#include "season/drive/AutoDrive.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "base/NtGamepad.h"
#include <base/season2026/MatchContext.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <networktables/NetworkTable.h>
#include <vector>
#include <string_view>
namespace season::runtime { class RobotRuntime; }
namespace season::autonomous { class RobotAutos; }
// Owns season subsystems and command bindings.
class RobotContainer
{
public:
  RobotContainer();
  ~RobotContainer();
  // Run mechanism feedback and control at the standard robot-loop cadence.
  void ControlMechanisms();
  frc2::Command *GetAutonomousCommand();
  void ScheduleAutonomous(frc2::Command *command);
  void CancelChecks();
  // Share one Driver Station-derived snapshot across season logic.
  void SetMatchContext(base::season2026::MatchContext context)
  {
    matchContext_ = std::move(context);
    if (autoDrive_)
      autoDrive_->PublishMatchContext();
  }
  void SimulationPeriodic();
  void PrepareAutonomous();
  // Autonomous waits until platform feedback is ready.
  bool AutonomousReady() const;

private:
  friend class season::runtime::RobotRuntime;
  friend class season::autonomous::RobotAutos;
  std::unique_ptr<season::runtime::RobotRuntime> runtime_;
  base::season2026::MatchContext matchContext_;
  base::Drive drive_;
  base::PositionJoint rack_, hood_;
  base::Flywheel intake_, conveyor_, bottom_, top_, shooter_;
  frc::AprilTagFieldLayout layout_;
  season::FieldGeometry field_;
  frc::Rotation2d driverHeadingOffset_{};
  double lastAcceptedVision_ = -1e9;
  std::unique_ptr<base::VisionSystem> vision_;
  void AcceptVision(const base::VisionObservation &observation);
  base::SystemCheck checks_;
  std::unique_ptr<season::AutoDrive> autoDrive_;
  std::shared_ptr<season::OffensiveFuelEstimate> fuelEstimate_ = std::make_shared<season::OffensiveFuelEstimate>();

  std::unique_ptr<base::NtGamepad> ntGamepad_;
  std::string_view activeDriverInput_ = "none";
  frc2::CommandXboxController controller_{season::hardware::kPilotControllerPort}, copilot_{season::hardware::kCopilotControllerPort};
  std::unique_ptr<season::autonomous::RobotAutos> autonomous_;
  season::Mechanisms Mechanisms();
  void ConfigureBindings();
  void DriveTeleop(bool aim);
};
