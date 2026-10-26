#pragma once
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/SeasonCommands.h"
#include "season/SystemCheck.h"
#include "season/FuelSimulation.h"
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
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
  void CancelChecks();
  void SimulationPeriodic();
  void ResetSimulation();
 private:
  base::Drive drive_;
  base::PositionJoint rack_,hood_;
  base::Flywheel intake_,conveyor_,bottom_,top_,shooter_;
  frc::AprilTagFieldLayout layout_;
  season::FieldGeometry field_;
  double lastAcceptedVision_ = -1e9;
  base::Vision vision_;
  season::SystemCheck checks_;
  std::unique_ptr<season::FuelSimulation> simulation_;
#ifndef __FRC_ROBORIO__
  nt::StructPublisher<frc::Pose3d> simulationPosePublisher_;
  int simulationResetFeedbackLoops_{};
#endif
  frc2::CommandXboxController controller_{0},copilot_{1};
  frc::SendableChooser<frc2::Command*> autoChooser_;
  std::vector<frc2::CommandPtr> seasonAutos_;
  season::Mechanisms Mechanisms();
  season::ShotTarget SelectTarget();
  frc2::CommandPtr HubShot(bool ownDrive=true,bool requireVision=false);
};
