#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "season/simulation/SimulationConfig.h"
#include "season/runtime/SeasonPolicy.h"
#include "season/RobotGeometry.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/shooting/Shooting.h"
#include <base/TelemetryPublishers.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>

namespace season
{
        namespace
        {
                std::vector<double> Positions(const std::vector<FuelParticle> &particles)
                {
                        std::vector<double> values;
                        values.reserve(particles.size() * 3);
                        for (const auto &p : particles)
                        {
                                values.push_back(p.position.x);
                                values.push_back(p.position.y);
                                values.push_back(p.position.z);
                        }
                        return values;
                }
                std::vector<double> Positions(const std::vector<FuelPoint> &particles)
                {
                        std::vector<double> values;
                        values.reserve(particles.size() * 3);
                        for (const auto &p : particles)
                        {
                                values.push_back(p.x);
                                values.push_back(p.y);
                                values.push_back(p.z);
                        }
                        return values;
                }

                meco::sim::FuelModelConfig SeasonFuelModelConfig()
                {
                        meco::sim::FuelModelConfig config;
                        auto &robot = config.robot;
                        constexpr double radiansPerDegree = kPi / 180;
                        constexpr double half = fuelconfig::kBumperHalfWidthMetres;
                        constexpr double depth = fuelconfig::kBumperDepthMetres;
                        constexpr double bottom = fuelconfig::kBumperBottomMetres;
                        constexpr double top = bottom + 2 * fuelconfig::kBumperHalfHeightMetres;
                        for (int side = 0; side < 4; ++side)
                        {
                                const bool alongX = side < 2;
                                const double sign = side % 2 ? 1 : -1;
                                robot.bumperBoxes.push_back(
                                    {{alongX ? sign * (half - depth / 2) : 0,
                                      alongX ? 0 : sign * (half - depth / 2), (bottom + top) / 2},
                                     {alongX ? depth / 2 : half, alongX ? half : depth / 2,
                                      (top - bottom) / 2}});
                        }
                        robot.capacity = fuelconfig::kSimulationCapacity;
                        robot.burstSize = fuelconfig::kSimulationBurstSize;
                        robot.hopperFloorMetres = fuelconfig::kHopperFloorMetres;
                        robot.hopperCeilingMetres = fuelconfig::kHopperCeilingMetres;
                        robot.hopperBackMetres = fuelconfig::kHopperBackMetres;
                        robot.hopperHalfWidthMetres = fuelconfig::kHopperHalfWidthMetres;
                        robot.hopperExtensionScale = kRackAngleCos;
                        robot.burstLateralCenter = fuelconfig::kBurstLateralCenter;
                        robot.burstLateralSpacingMetres = fuelconfig::kBurstLateralSpacingMetres;
                        robot.minHoodDeflectionRadians = fuelconfig::kMinHoodDeflectionDegrees * radiansPerDegree;
                        robot.maxHoodDeflectionRadians = fuelconfig::kMaxHoodDeflectionDegrees * radiansPerDegree;
                        robot.shooterRestDeflectionRadians = kShooterRestDeflectionDegrees * radiansPerDegree;
                        robot.minLaunchPitchRadians = fuelconfig::kMinLaunchPitchDegrees * radiansPerDegree;
                        robot.maxLaunchPitchRadians = fuelconfig::kMaxLaunchPitchDegrees * radiansPerDegree;
                        robot.launchPitchNoiseRadians = fuelconfig::kLaunchPitchNoiseDegrees * radiansPerDegree;
                        robot.launchSpeedNoiseFraction = fuelconfig::kLaunchSpeedNoiseFraction;
                        robot.shooterExitXMetres = kShooterExitXMetres;
                        robot.releaseHeightMetres = kReleaseHeight;
                        robot.projectileSpeedPerFlywheelRps = kProjectileSpeedPerRps;
                        robot.collectionMinExtensionMetres = fuelconfig::kCollectionMinRackMetres;
                        robot.collectionMouthXMetres = fuelconfig::kCollectionMouthXMetres;
                        robot.collectionMouthDepthMetres = fuelconfig::kCollectionMouthDepthMetres;
                        robot.collectionHalfWidthMetres = fuelconfig::kCollectionHalfWidthMetres;
                        robot.minFeedFlywheelRps = fuelconfig::kMinFeedFlywheelRps;
                        robot.burstResetPeriods = fuelconfig::kBurstResetPeriods;
                        robot.minLaunchGapSeconds = fuelconfig::kMinLaunchGapSeconds;
                        robot.launchGapSeconds = fuelconfig::kLaunchGapSeconds;
                        robot.launchGapNoiseSeconds = fuelconfig::kLaunchGapNoiseSeconds;
                        robot.burstPeriodSeconds = fuelconfig::kSimulationBurstPeriodSeconds;
                        return config;
                }
        }
        FuelSimulation::FuelSimulation(Mechanisms mechanisms, base::Drive &drive, std::shared_ptr<FuelMotorLoads> loads) : mechanisms_(mechanisms), drive_(drive), model_(simulation::runtimeconfig::kFuelRandomSeed, SeasonFuelModelConfig()), loads_(std::move(loads))
        {
                auto nt = nt::NetworkTableInstance::GetDefault();
                robotPosePublisher_ = std::make_shared<nt::StructPublisher<frc::Pose3d>>(
                    nt.GetStructTopic<frc::Pose3d>("/Visualization/RobotRemy/RobotPose").Publish());
                componentsPublisher_ = std::make_shared<nt::StructArrayPublisher<frc::Pose3d>>(
                    nt.GetStructArrayTopic<frc::Pose3d>("/Visualization/RobotRemy/ComponentPoses").Publish());
                fuelPublisher_ = std::make_shared<nt::StructArrayPublisher<frc::Pose3d>>(
                    nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelPositions").Publish());
                hopperPublisher_ = std::make_shared<nt::StructArrayPublisher<frc::Pose3d>>(
                    nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/HopperFuelPositions").Publish());
                projectilesPublisher_ = std::make_shared<nt::StructArrayPublisher<frc::Pose3d>>(
                    nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelProjectilePositions").Publish());
                Reset();
        }
        void FuelSimulation::Reset()
        {
                model_.Reset();
                modelTime_ = 0;
                lastHubActive_.reset();
                scoringGraceUntil_.reset();
                shooterLoadRemaining_ = 0;
                loadRandom_.seed(simulation::runtimeconfig::kFuelRandomSeed);
                if (loads_)
                        loads_->torque.fill(0);
                // Deterministic neutral-zone grid, rather than claiming official Maple arena placement.
                for (double x = fuelconfig::kStartingFuelMinX; x <= fuelconfig::kStartingFuelMaxX; x += fuelconfig::kStartingFuelSpacing)
                        for (double y = fuelconfig::kStartingFuelMinY; y <= fuelconfig::kStartingFuelMaxY; y += fuelconfig::kStartingFuelSpacing)
                                model_.AddFloorFuel({x, y, 0});
        }
        void FuelSimulation::Periodic(double dt, std::optional<FuelRobotState> physics, std::optional<bool> hubActive,
                                      std::optional<frc::DriverStation::Alliance> alliance)
        {
                auto pose = drive_.Pose();
                auto speed = drive_.Speeds();
                double heading = pose.Rotation().Radians().value(), c = std::cos(heading), s = std::sin(heading);
                meco::sim::FuelModelInput input;
                input.robot = physics.value_or(FuelRobotState{
                    frc::Pose3d{pose},
                    {speed.vx.value() * c - speed.vy.value() * s,
                     speed.vx.value() * s + speed.vy.value() * c, 0},
                    {0, 0, speed.omega.value()}, {}});
                input.intakeExtensionMetres = mechanisms_.rack.Position();
                input.hoodRotations = mechanisms_.hood.Position();
                input.flywheelRps = mechanisms_.shooter.Velocity();
                input.collecting = mechanisms_.intake.Inputs().voltage < -simulationconfig::kSimTransportVoltageThreshold && mechanisms_.conveyor.Inputs().voltage > simulationconfig::kSimTransportVoltageThreshold && mechanisms_.intake.Velocity() < 0 && mechanisms_.conveyor.Velocity() > 0;
                input.feeding = simulation::TransportFeeding(mechanisms_);
                input.redAlliance = alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
                                    frc::DriverStation::Alliance::kRed;
                if (std::isfinite(dt) && dt > 0 && dt <= model_.Config().game.maxStepSeconds)
                {
                        modelTime_ += dt;
                        if (!hubActive)
                        {
                                lastHubActive_.reset();
                                scoringGraceUntil_.reset();
                        }
                        else
                        {
                                if (!*hubActive && lastHubActive_.value_or(false))
                                        scoringGraceUntil_ = modelTime_ + fuelconfig::kScoringGraceSeconds;
                                if (*hubActive)
                                        scoringGraceUntil_.reset();
                                lastHubActive_ = hubActive;
                        }
                        input.scoringEligible = hubActive &&
                                                (*hubActive || (scoringGraceUntil_ && modelTime_ <= *scoringGraceUntil_));
                }
                base::telemetry::PostDashboardBoolean("FieldSimulation/Feeding", input.feeding);
                base::telemetry::PostDashboardNumber("FieldSimulation/BottomIndexerVolts", mechanisms_.bottomIndexer.Inputs().voltage);
                base::telemetry::PostDashboardNumber("FieldSimulation/TopIndexerVolts", mechanisms_.topIndexer.Inputs().voltage);
                base::telemetry::PostDashboardNumber("FieldSimulation/ConveyorVolts", mechanisms_.conveyor.Inputs().voltage);
                base::telemetry::PostDashboardNumber("FieldSimulation/ShooterCurrentAmps", mechanisms_.shooter.Inputs().current);
                base::telemetry::PostDashboardNumber("FieldSimulation/ConveyorCurrentAmps", mechanisms_.conveyor.Inputs().current);
                base::telemetry::PostDashboardNumber("FieldSimulation/BottomIndexerCurrentAmps", mechanisms_.bottomIndexer.Inputs().current);
                base::telemetry::PostDashboardNumber("FieldSimulation/TopIndexerCurrentAmps", mechanisms_.topIndexer.Inputs().current);
                const double fill = static_cast<double>(model_.StoredFuel()) / fuelconfig::kSimulationCapacity;
                const bool fuelInTransport = (input.feeding || input.collecting) && fill > 0;
                const int priorLaunches = model_.LaunchedFuel();
                model_.Step(dt, input);
                if (loads_)
                {
                        // Approximate fill-dependent, noisy fuel contact and shot impulses.
                        // These torque coefficients require hardware calibration.
                        if (model_.LaunchedFuel() > priorLaunches)
                                shooterLoadRemaining_ = fuelconfig::kShooterLoadDurationSeconds;
                        else
                                shooterLoadRemaining_ = std::max(0., shooterLoadRemaining_ - dt);
                        const auto noise = [this]
                        { return std::uniform_real_distribution<double>{fuelconfig::kLoadNoiseMin, fuelconfig::kLoadNoiseMax}(loadRandom_); };
                        const double load = fuelInTransport ? fuelconfig::kTransportBaseLoad + fuelconfig::kTransportFillLoad * fill : 0.;
                        loads_->torque = {fuelconfig::kConveyorLoadTorque * load * noise(), fuelconfig::kIndexerLoadTorque * load * noise(), fuelconfig::kIndexerLoadTorque * load * noise(), shooterLoadRemaining_ > 0 ? fuelconfig::kShooterLoadTorque * noise() : 0.};
                }
                const auto robotPose = input.robot.pose;
                auto robotPublisher = robotPosePublisher_;
                base::telemetry::Post("Simulation/RobotPose", [robotPublisher, robotPose]
                                      { robotPublisher->Set(robotPose); });
                auto componentsPublisher = componentsPublisher_;
                auto components = RemyComponentPoses(input.intakeExtensionMetres, input.hoodRotations, mechanisms_.shooter.Position());
                base::telemetry::Post("Simulation/RobotComponents", [componentsPublisher, components = std::move(components)]
                                      { componentsPublisher->Set(components); });
                auto hopper = model_.HopperCenters(model_.StoredFuel(), input.intakeExtensionMetres, robotPose);
                std::vector<frc::Pose3d> fuel, hopperPoses, projectiles;
                fuel.reserve(model_.FloorFuel().size() + hopper.size());
                hopperPoses.reserve(hopper.size());
                projectiles.reserve(model_.Projectiles().size());
                auto ballPose = [](FuelPoint p)
                { return frc::Pose3d{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z}, frc::Rotation3d{}}; };
                for (const auto &p : model_.FloorFuel())
                        fuel.push_back(ballPose(p.position));
                for (const auto &p : hopper)
                        hopperPoses.push_back(ballPose(p));
                fuel.insert(fuel.end(), hopperPoses.begin(), hopperPoses.end());
                for (const auto &p : model_.Projectiles())
                        projectiles.push_back(ballPose(p.position));
                auto fuelPublisher = fuelPublisher_;
                base::telemetry::Post("Simulation/FuelPositions", [fuelPublisher, fuel = std::move(fuel)]
                                      { fuelPublisher->Set(fuel); });
                auto hopperPublisher = hopperPublisher_;
                base::telemetry::Post("Simulation/HopperFuelPositions", [hopperPublisher, hopperPoses = std::move(hopperPoses)]
                                      { hopperPublisher->Set(hopperPoses); });
                auto projectilesPublisher = projectilesPublisher_;
                base::telemetry::Post("Simulation/FuelProjectilePositions", [projectilesPublisher, projectiles = std::move(projectiles)]
                                      { projectilesPublisher->Set(projectiles); });
                base::telemetry::PostDashboardNumber("FieldSimulation/StoredFuelCount", model_.StoredFuel());
                base::telemetry::PostDashboardNumber("FieldSimulation/HopperFuelCount", hopper.size());
                base::telemetry::PostDashboardNumber("FieldSimulation/HopperFillRatio", hopper.empty() ? 0 : static_cast<double>(hopper.size()) / model_.HopperCenters(fuelconfig::kSimulationCapacity, input.intakeExtensionMetres, robotPose).size());
                auto fuelPositions = Positions(model_.FloorFuel());
                const auto hopperPositions = Positions(hopper);
                fuelPositions.insert(fuelPositions.end(), hopperPositions.begin(), hopperPositions.end());
                base::telemetry::PostDashboardNumber("FieldSimulation/SuccessfulScoreCount", model_.SuccessfulScores());
                base::telemetry::PostDashboardNumber("FieldSimulation/LaunchedFuelCount", model_.LaunchedFuel());
                base::telemetry::PostDashboardNumber("FieldSimulation/FuelProjectileCount", model_.Projectiles().size());
                base::telemetry::PostDashboardNumberArray("FieldSimulation/FuelPositionsXYZ", fuelPositions);
                base::telemetry::PostDashboardNumberArray("FieldSimulation/HopperFuelPositionsXYZ", hopperPositions);
                base::telemetry::PostDashboardNumberArray("FieldSimulation/FuelProjectilePositionsXYZ", Positions(model_.Projectiles()));
        }
}
#endif
