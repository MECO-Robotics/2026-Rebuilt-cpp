#pragma once
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "base/QuestNav.h"
#include "season/simulation/Simulation.h"
#include "season/mechanisms/Mechanisms.h"
#include "base/SystemCheck.h"
#include "season/drive/AutoDrive.h"
#include "season/drive/OffensiveAutoDrive.h"
#include "season/drive/NtGamepad.h"
#include "season/drive/FastControlProfile.h"
#include <base/season2026/MatchContext.h>
#ifndef __FRC_ROBORIO__
#include "meco/sim/DriveSimulationAdapter.h"

#endif
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <networktables/NetworkTable.h>
#include <networktables/StructTopic.h>
#ifndef __FRC_ROBORIO__
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#endif
#include <vector>
#include <cstdint>
// Owns season subsystems, command bindings, and simulation adapters.
class RobotContainer
{
public:
  RobotContainer();
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
  bool SimulationResetReady() const
  {
#ifndef __FRC_ROBORIO__
    return simulationResetFeedbackLoops_ == 0;
#else
    return true;
#endif
  }

private:
  base::season2026::MatchContext matchContext_;
  base::Drive drive_;
#ifndef __FRC_ROBORIO__
  std::shared_ptr<season::FuelMotorLoads> fuelMotorLoads_ = std::make_shared<season::FuelMotorLoads>();
#endif
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
#ifndef __FRC_ROBORIO__
  season::simulation::SimulatedLocalization simulatedLocalization_;
#endif
  std::unique_ptr<base::VisionSystem> vision_;
  std::vector<std::unique_ptr<base::VisionIO>> LocalizationSources();
  void AcceptVision(const base::VisionObservation &observation);
  base::SystemCheck checks_;
  std::unique_ptr<season::AutoDrive> autoDrive_;
  std::shared_ptr<season::OffensiveFuelEstimate> fuelEstimate_ = std::make_shared<season::OffensiveFuelEstimate>();

#ifndef __FRC_ROBORIO__
  std::unique_ptr<season::FuelSimulation> simulation_;
  nt::DoubleArrayPublisher ntGamepadConsumedPublisher_;
  nt::DoubleArrayPublisher driverStationCommandPublisher_;
  meco::sim::DriveSimulationSession *simulationSession_{}; // Non-owning; drive_ owns it.
  nt::StructPublisher<frc::Pose3d> simulationPosePublisher_;
  nt::StructPublisher<frc::Pose3d> simulationBodyPosePublisher_;
  units::meter_t simulationGroundedBodyHeight_{};
  int simulationResetFeedbackLoops_{};
  frc::Pose2d simulationResetPose_{};
  frc::Pose2d simulationResetPhysicalOrigin_{};
  bool simulationFieldCollidersEnabled_{true};
  bool captureAutonomousPoseReset_ = false;
  bool autonomousPoseBlocked_ = false;
#endif
  std::unique_ptr<season::drive::NtGamepad> ntGamepad_;
  frc2::CommandXboxController controller_{0}, copilot_{1};
  frc::SendableChooser<frc2::Command *> autoChooser_;
  std::vector<frc2::CommandPtr> seasonAutos_;
  season::Mechanisms Mechanisms();
  season::ShotTarget SelectTarget();
  frc2::CommandPtr HubShot(bool ownDrive = true, bool requireVision = false);
  void ConfigureBindings();
  void DriveTeleop(bool aim);
};
