#pragma once
#include "base/Mechanism.h"
#include "base/Vision.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/mechanisms/MechanismConfig.h"
#include "season/drive/NtGamepad.h"
#include "season/drive/FastControlProfile.h"
#include <memory>
#include <vector>
#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "meco/sim/DriveSimulationAdapter.h"
#include <networktables/StructTopic.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#endif
class RobotContainer;
namespace season::simulation {
enum class MotorLoad { None, Conveyor, Bottom, Top, Shooter };
class RobotRuntime {
public:
  explicit RobotRuntime(RobotContainer &owner) : owner_(owner) {}
  std::unique_ptr<base::MotorIO> MakeMotor(const season::hardware::FlywheelConfig &, base::Vendor, MotorLoad loadedMotor);
  std::vector<std::unique_ptr<base::VisionIO>> LocalizationSources();
  void Initialize();
  void InitializeFuel();
  void ResetIfSimulation();
  bool IsSimulation() const;
  bool UseNtInput(bool selected) const;
  void PublishDriverInput(const season::drive::NtGamepad::State &, double, double, double, double, double, double);
  const std::vector<int> *TagIds() const;
  void BindReset();
  void Schedule(frc2::Command *);
  void Periodic();
  void Reset();
  bool ResetReady() const;
private:
  RobotContainer &owner_;
#ifndef __FRC_ROBORIO__
  std::shared_ptr<season::FuelMotorLoads> fuelMotorLoads_ = std::make_shared<season::FuelMotorLoads>();
  season::simulation::SimulatedLocalization simulatedLocalization_;
  std::unique_ptr<season::FuelSimulation> fuelSimulation_;
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
 };
}
