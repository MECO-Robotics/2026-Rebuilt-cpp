#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/ParticleContacts.h"
#include "season/SeasonCommands.h"
#include <random>
#include <optional>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <vector>
namespace season
{
  using FuelPoint = meco::sim::ParticlePoint;
  using FuelParticle = meco::sim::Particle;
  using FuelRobotState = meco::sim::ContactBody;
  struct FuelInputs
  {
    Pose robot;
    // Field relative chassis velocity, including launch inheritance.
    double vx{}, vy{}, omega{}, rackMetres{}, hoodRotations{}, flywheelRps{};
    bool collecting = false, feeding = false;
    Alliance alliance = Alliance::Blue;
    std::optional<FuelRobotState> physics;
  };
  // Gravity-driven balls with estimated bumper and equal-mass fuel contacts.
  // Robot motion is prescribed (no fuel reaction force on the drivetrain).
  // No field-structure contacts, drag, spin forces, or flywheel load.
  // Seeded launch randomness is repeatable.
  class FuelModel
  {
  public:
    explicit FuelModel(unsigned seed = 3737);
    void Reset();
    void Step(double dt, const FuelInputs &input);
    void AddFloorFuel(Point point);
    void SetStoredFuel(int count);
    int StoredFuel() const { return stored_; }
    int SuccessfulScores() const { return scored_; }
    int LaunchedFuel() const { return launched_; }
    int PendingBurstShots() const { return pending_; }
    const std::vector<FuelParticle> &FloorFuel() const { return floor_; }
    const std::vector<FuelParticle> &Projectiles() const { return airborne_; }
    static std::vector<FuelPoint> HopperCenters(int count, double extension, Pose robot = {});
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
  };
  // Call only from Robot::SimulationPeriodic. Actuator voltages select collection
  // and feeding, so command cancellation immediately stops simulated launching.
  class FuelSimulation
  {
  public:
    FuelSimulation(Mechanisms mechanisms, base::Drive &drive);
    void Periodic(double dt = .02, std::optional<FuelRobotState> physics = std::nullopt);
    void Reset();
    FuelModel &Model() { return model_; }

  private:
    Mechanisms mechanisms_;
    base::Drive &drive_;
    FuelModel model_;
    nt::StructPublisher<frc::Pose3d> robotPosePublisher_;
    nt::StructArrayPublisher<frc::Pose3d> componentsPublisher_;
    nt::StructArrayPublisher<frc::Pose3d> fuelPublisher_, hopperPublisher_, projectilesPublisher_;
  };
}

#endif
