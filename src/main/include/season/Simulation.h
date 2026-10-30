#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/ParticleContacts.h"
#include "season/Mechanisms.h"
#include <random>
#include <optional>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <vector>
#include "base/Vision.h"
#include "meco/sim/IdealVisionObservation.h"
#include "meco/sim/SimulatedVision.h"
#include <frc/geometry/Pose3d.h>
#include "base/sim/Arena.h"
#include "base/sim/fields/Rebuilt2026.h"
#include "base/FootprintSpeedProfile.h"
#include <frc/geometry/Pose2d.h>

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
  // No field-structure contacts, drag, spin forces, or flywheel load.
  // Seeded launch randomness is repeatable.
  class FuelModel
  {
  public:
    explicit FuelModel(unsigned seed = 3737);
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
    FuelSimulation(Mechanisms mechanisms, base::Drive &drive);
    void Periodic(double dt = .02, std::optional<FuelRobotState> physics = std::nullopt,
                  std::optional<bool> hubActive = true);
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



namespace season::simulation
{
    // Robot-specific ideal sensor model. Geometry/mounts are supplied by the season;
    // shared acquisition, filtering and estimator ownership remain in MecoBase.
    struct SimulatedLocalization
    {
        base::VisionInputs limelight, quest;
        std::vector<int> tagIds;
        bool questAligned = false;

        void Reset()
        {
            limelight = {};
            quest = {};
            tagIds.clear();
            questAligned = false;
        }

        void Update(const frc::Pose3d &truth, units::second_t timestamp,
                    const frc::AprilTagFieldLayout &layout,
                    const frc::Transform3d &limelightMount, const frc::Transform3d &questMount)
        {
            limelight = {};
            quest = {};
            limelight.connected = quest.connected = quest.tracking = true;
            // LL3 standard lens; Quest optical frustum is an approximate model.
            tagIds = meco::sim::VisibleTags(truth, limelightMount, layout,
                                            units::degree_t{62.5}, units::degree_t{48.9}, units::meter_t{6});
            const auto questTags = meco::sim::VisibleTags(truth, questMount, layout,
                                                          units::degree_t{90}, units::degree_t{90}, units::meter_t{6});
            // Model AprilTag-assisted absolute initialization, then continuous VIO.
            questAligned = questAligned || questTags.size() >= 2 || !tagIds.empty();
            quest.aligned = questAligned;
            const auto limelightObservation = meco::sim::MakeIdealVisionObservation(
                truth, timestamp, layout, !tagIds.empty(), units::meter_t{.75}, {.08, .08, .12});
            const auto questObservation = meco::sim::MakeIdealVisionObservation(
                truth, timestamp, layout, questAligned, units::meter_t{.75}, {.1, .1, .05});
            if (limelightObservation)
                limelight.observations.push_back({limelightObservation->pose, limelightObservation->timestamp, limelightObservation->stdDevs});
            if (questObservation)
                quest.observations.push_back({questObservation->pose, questObservation->timestamp, questObservation->stdDevs});
        }
    };
}


namespace season::simulation
{
    using base::sim::fields::rebuilt2026::FieldColliders;
    using base::sim::fields::rebuilt2026::kBlueHubX;
    using base::sim::fields::rebuilt2026::kFieldLength;
    using base::sim::fields::rebuilt2026::kFieldWidth;
    using base::sim::fields::rebuilt2026::kHubY;
    using base::sim::fields::rebuilt2026::kRedHubX;
    using base::sim::fields::rebuilt2026::kStartingLineX;
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
