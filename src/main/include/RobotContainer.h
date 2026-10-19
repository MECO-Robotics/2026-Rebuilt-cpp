#pragma once
#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "base/QuestNav.h"
#include "season/SeasonCommands.h"
#include "season/SystemCheck.h"
#include "season/FuelSimulation.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <networktables/NetworkTable.h>
#include <vector>
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
  void CancelChecks();
  void Periodic();
  void ResetLocalization(const frc::Pose2d& pose);
  void SimulationPeriodic();
  void ResetSimulation();
 private:
  base::Drive drive_;
  base::PositionJoint rack_,hood_;
  base::Flywheel intake_,conveyor_,bottom_,top_,shooter_;
  frc::AprilTagFieldLayout layout_;
  season::FieldGeometry field_;
  double lastAcceptedVision_ = -1e9;
  std::optional<base::VisionObservation> absoluteVision_;
  std::array<base::VisionObservation, 20> pendingAbsolute_{};
  std::size_t pendingAbsoluteCount_ = 0;
  std::size_t pendingAbsoluteStart_ = 0;
  unsigned droppedAbsolute_ = 0;
  base::QuestNav quest_;
  base::QuestNavTransport questTransport_;
  double lastQuestDiagnostic_ = -1e9;
  base::Vision vision_;
  season::SystemCheck checks_;
  std::unique_ptr<season::FuelSimulation> simulation_;
  frc2::CommandXboxController controller_{0},copilot_{1};
  frc::SendableChooser<frc2::Command*> autoChooser_;
  std::vector<frc2::CommandPtr> seasonAutos_;
  season::Mechanisms Mechanisms();
  bool VisionConnected() const;
  season::ShotTarget SelectTarget();
  frc2::CommandPtr HubShot(bool ownDrive=true,bool requireVision=false);
};
