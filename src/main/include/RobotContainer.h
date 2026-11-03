#pragma once
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/RobotConfiguration.h"
#include "base/SystemCheck.h"
#include "season/drive/AutoDrive.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "season/drive/NtGamepad.h"
#include "season/drive/FastControlProfile.h"
#include <base/season2026/MatchContext.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <networktables/NetworkTable.h>
#include <vector>
#include <cstdint>
#include <string_view>
namespace season::simulation { class RobotRuntime; }
namespace season::autonomous { class RobotAutos; }
// Owns season subsystems and command bindings.
class RobotContainer
{
public:
  RobotContainer();
  ~RobotContainer();
  // Run the mechanisms' control loops at their original 20 ms cadence when
  // the command scheduler is configured to run faster than the TimedRobot loop.
  void ControlMechanisms20ms();
  // Run the selected mechanism groups at their opt-in fast-control periods.
  void ControlMechanismsFast();
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
  void ResetSimulation();
  // Autonomous waits until reset feedback has been reconciled.
  bool SimulationResetReady() const;

private:
  friend class season::simulation::RobotRuntime;
  friend class season::autonomous::RobotAutos;
  std::unique_ptr<season::simulation::RobotRuntime> simulationRuntime_;
  base::season2026::MatchContext matchContext_;
  base::Drive drive_;
  base::PositionJoint rack_, hood_;
  base::Flywheel intake_, conveyor_, bottom_, top_, shooter_;
  frc::AprilTagFieldLayout layout_;
  season::FieldGeometry field_;
  frc::Rotation2d driverHeadingOffset_{};
  double lastAcceptedVision_ = -1e9;
  bool fastControlEnabled_{};
  season::drive::fastcontrol::Profile fastControlProfile_ = season::drive::fastcontrol::Config();
  bool fastMechanismCanFd_{};
  uint64_t fastMechanismControlTick_{};
  uint64_t sparkControlCount_{}, talonControlCount_{};
  std::unique_ptr<base::VisionSystem> vision_;
  void AcceptVision(const base::VisionObservation &observation);
  base::SystemCheck checks_;
  std::unique_ptr<season::AutoDrive> autoDrive_;
  std::shared_ptr<season::OffensiveFuelEstimate> fuelEstimate_ = std::make_shared<season::OffensiveFuelEstimate>();

  std::unique_ptr<season::drive::NtGamepad> ntGamepad_;
  std::string_view activeDriverInput_ = "none";
  frc2::CommandXboxController controller_{season::hardware::kPilotControllerPort}, copilot_{season::hardware::kCopilotControllerPort};
  std::unique_ptr<season::autonomous::RobotAutos> autonomous_;
  season::Mechanisms Mechanisms();
  void ConfigureBindings();
  void DriveTeleop(bool aim);
};
