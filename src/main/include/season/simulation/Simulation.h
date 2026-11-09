#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/ParticleContacts.h"
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
#include "season/simulation/FieldGeometry2026.h"
#include "base/FootprintSpeedProfile.h"
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>

namespace season
{
  using FuelPoint = meco::sim::ParticlePoint;
  using FuelParticle = meco::sim::Particle;
  using FuelRobotState = meco::sim::ContactBody;
  struct FuelInputs
  {
    frc::Pose2d robot;
    // Field relative chassis velocity, including launch inheritance.
    double vx{}, vy{}, omega{}, rackMetres{}, hoodRotations{}, flywheelRps{};
    bool collecting = false, feeding = false;
    // Scoring depends on hub state at the moment fuel crosses the opening.
    // A missing state cannot establish that a crossing earned points.
    std::optional<bool> hubActive{true};
    frc::DriverStation::Alliance alliance = frc::DriverStation::Alliance::kBlue;
    std::optional<FuelRobotState> physics;
  };
  // Gravity-driven balls with estimated bumper and equal-mass fuel contacts.
  // Robot motion is prescribed (no fuel reaction force on the drivetrain).
  // Field contacts are approximate; drag and spin forces are omitted.
  // Seeded launch randomness is repeatable.
  class FuelModel
  {
  public:
    explicit FuelModel(unsigned seed = simulation::runtimeconfig::kFuelRandomSeed);
    void Reset();
    void Step(double dt, const FuelInputs &input);
    void AddFloorFuel(frc::Translation2d point);
    void SetStoredFuel(int count);
    int StoredFuel() const { return stored_; }
    int SuccessfulScores() const { return scored_; }
    int LaunchedFuel() const { return launched_; }
    int PendingBurstShots() const { return pending_; }
    const std::vector<FuelParticle> &FloorFuel() const { return floor_; }
    const std::vector<FuelParticle> &Projectiles() const { return airborne_; }
    static std::vector<FuelPoint> HopperCenters(int count, double extension, frc::Pose2d robot = {});
    static std::vector<FuelPoint> HopperCenters3d(int count, double extension, const frc::Pose3d &robot);
    static double LaunchPitch(double hoodRotations);

  private:
    void Launch(const FuelInputs &input, int index);
    double Random(double amplitude);
    unsigned seed_;
    std::mt19937 random_;
    std::vector<FuelParticle> floor_, airborne_;
    int stored_ = 0, scored_ = 0, launched_ = 0, pending_ = 0, burstIndex_ = 0;
    double time_ = 0, lastBurst_ = -1e9, nextShot_ = -1e9;
    std::optional<bool> lastHubActive_;
    std::optional<double> scoringGraceUntil_;
  };
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
