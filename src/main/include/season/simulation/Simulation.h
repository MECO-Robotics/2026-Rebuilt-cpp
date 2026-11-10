#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/FuelModel.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/simulation/SimulationConfig.h"
#include "season/simulation/MotorSimulation.h"
#include <array>
#include <memory>
#include <random>
#include <optional>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <vector>
#include "base/Vision.h"
#include <frc/geometry/Pose3d.h>
#include "base/sim/Arena.h"
#include "meco/sim/Field2026.h"
#include "base/FootprintSpeedProfile.h"
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>

namespace season
{
  using FuelPoint = meco::sim::ParticlePoint;
  using FuelParticle = meco::sim::Particle;
  using FuelRobotState = meco::sim::ContactBody;
  using FuelModel = meco::sim::FuelModel;
  // Call only from Robot::SimulationPeriodic. Actuator voltages select collection
  // and feeding, so command cancellation immediately stops simulated launching.
  class FuelSimulation
  {
  public:
    FuelSimulation(Mechanisms mechanisms, base::Drive &drive, std::shared_ptr<FuelMotorLoads> loads = {});
    void Periodic(double dt = simulation::runtimeconfig::kSimulationPeriodSeconds, std::optional<FuelRobotState> physics = std::nullopt,
                  std::optional<bool> hubActive = true,
                  std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::Alliance::kBlue);
    void Reset();
    FuelModel &Model() { return model_; }

  private:
    Mechanisms mechanisms_;
    base::Drive &drive_;
    FuelModel model_;
    std::shared_ptr<FuelMotorLoads> loads_;
    double shooterLoadRemaining_{};
    double modelTime_{};
    std::optional<bool> lastHubActive_;
    std::optional<double> scoringGraceUntil_;
    std::mt19937 loadRandom_{simulation::runtimeconfig::kFuelRandomSeed};
    std::shared_ptr<nt::StructPublisher<frc::Pose3d>> robotPosePublisher_;
    std::shared_ptr<nt::StructArrayPublisher<frc::Pose3d>> componentsPublisher_;
    std::shared_ptr<nt::StructArrayPublisher<frc::Pose3d>> fuelPublisher_, hopperPublisher_, projectilesPublisher_;
  };
}

namespace season::simulation
{
  // Season-specific transport feedback used by command policy and fuel motion.
  bool TransportFeeding(const Mechanisms &mechanisms);

}

namespace season::simulation
{
  frc::Pose2d DefaultSpawn(bool red);
  // Conservative clearance of the modeled .86m square chassis up to .45m high.
  // This is a modeled start-space check, not whole-robot inspection acceptance.
  bool IsClearStartingPose(const frc::Pose2d &pose);
  void InstallField(base::sim::Arena &arena);
  // Centerline terrain for the modeled pitched plates; rolling resistance is
  // not modeled here. These functions are desktop simulation inputs only.
  std::vector<double> RampBoundaries(base::FootprintPose a, base::FootprintPose b);
  base::FootprintTerrainSegment RampTerrain(base::FootprintPose a, base::FootprintPose b);
  // Conservative plan-view rectangles for the optional ramp avoidance policy.
  std::vector<base::FieldRectangle> RampObstacles();
}
#endif
