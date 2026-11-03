#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "season/simulation/SimulationConfig.h"
#include "season/RobotGeometry.h"
#include "season/mechanisms/Mechanisms.h"
#include "meco/sim/ParticleFieldContacts.h"
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
        }
        FuelSimulation::FuelSimulation(Mechanisms mechanisms, base::Drive &drive, std::shared_ptr<FuelMotorLoads> loads) : mechanisms_(mechanisms), drive_(drive), loads_(std::move(loads))
        {
                auto nt = nt::NetworkTableInstance::GetDefault();
                robotPosePublisher_ = nt.GetStructTopic<frc::Pose3d>("/Visualization/RobotRemy/RobotPose").Publish();
                componentsPublisher_ = nt.GetStructArrayTopic<frc::Pose3d>("/Visualization/RobotRemy/ComponentPoses").Publish();
                fuelPublisher_ = nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelPositions").Publish();
                hopperPublisher_ = nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/HopperFuelPositions").Publish();
                projectilesPublisher_ = nt.GetStructArrayTopic<frc::Pose3d>("/FieldSimulation/FuelProjectilePositions").Publish();
                Reset();
        }
        void FuelSimulation::Reset()
        {
                model_.Reset();
                shooterLoadRemaining_ = 0;
                loadRandom_.seed(simulation::runtimeconfig::kFuelRandomSeed);
                if (loads_)
                        loads_->torque.fill(0);
                // Deterministic neutral-zone grid, rather than claiming official Maple arena placement.
                for (double x = fuelconfig::kStartingFuelMinX; x <= fuelconfig::kStartingFuelMaxX; x += fuelconfig::kStartingFuelSpacing)
                        for (double y = fuelconfig::kStartingFuelMinY; y <= fuelconfig::kStartingFuelMaxY; y += fuelconfig::kStartingFuelSpacing)
                                model_.AddFloorFuel({units::meter_t{x}, units::meter_t{y}});
        }
        void FuelSimulation::Periodic(double dt, std::optional<FuelRobotState> physics, std::optional<bool> hubActive)
        {
                auto pose = drive_.Pose();
                auto speed = drive_.Speeds();
                double heading = pose.Rotation().Radians().value(), c = std::cos(heading), s = std::sin(heading);
                FuelInputs input;
                input.robot = pose;
                input.vx = speed.vx.value() * c - speed.vy.value() * s;
                input.vy = speed.vx.value() * s + speed.vy.value() * c;
                input.omega = speed.omega.value();
                input.rackMetres = mechanisms_.rack.Position();
                input.hoodRotations = mechanisms_.hood.Position();
                input.flywheelRps = mechanisms_.shooter.Velocity();
                input.collecting = mechanisms_.intake.Inputs().voltage < -hardware::kSimTransportVoltageThreshold && mechanisms_.conveyor.Inputs().voltage > hardware::kSimTransportVoltageThreshold && mechanisms_.intake.Velocity() < 0 && mechanisms_.conveyor.Velocity() > 0;
                input.feeding = SimulatedTransportFeeding(mechanisms_);
                input.physics = physics;
                input.hubActive = hubActive;
                frc::SmartDashboard::PutBoolean("FieldSimulation/Feeding", input.feeding);
                frc::SmartDashboard::PutNumber("FieldSimulation/BottomIndexerVolts", mechanisms_.bottomIndexer.Inputs().voltage);
                frc::SmartDashboard::PutNumber("FieldSimulation/TopIndexerVolts", mechanisms_.topIndexer.Inputs().voltage);
                frc::SmartDashboard::PutNumber("FieldSimulation/ConveyorVolts", mechanisms_.conveyor.Inputs().voltage);
                frc::SmartDashboard::PutNumber("FieldSimulation/ShooterCurrentAmps", mechanisms_.shooter.Inputs().current);
                frc::SmartDashboard::PutNumber("FieldSimulation/ConveyorCurrentAmps", mechanisms_.conveyor.Inputs().current);
                frc::SmartDashboard::PutNumber("FieldSimulation/BottomIndexerCurrentAmps", mechanisms_.bottomIndexer.Inputs().current);
                frc::SmartDashboard::PutNumber("FieldSimulation/TopIndexerCurrentAmps", mechanisms_.topIndexer.Inputs().current);
                input.alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
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
                const auto robotPose = input.physics ? input.physics->pose : frc::Pose3d{input.robot};
                robotPosePublisher_.Set(robotPose);
                componentsPublisher_.Set(RemyComponentPoses(input.rackMetres, input.hoodRotations, mechanisms_.shooter.Position()));
                auto hopper = FuelModel::HopperCenters3d(model_.StoredFuel(), input.rackMetres, robotPose);
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
                fuelPublisher_.Set(fuel);
                hopperPublisher_.Set(hopperPoses);
                projectilesPublisher_.Set(projectiles);
                frc::SmartDashboard::PutNumber("FieldSimulation/StoredFuelCount", model_.StoredFuel());
                frc::SmartDashboard::PutNumber("FieldSimulation/HopperFuelCount", hopper.size());
                frc::SmartDashboard::PutNumber("FieldSimulation/HopperFillRatio", hopper.empty() ? 0 : static_cast<double>(hopper.size()) / FuelModel::HopperCenters(fuelconfig::kSimulationCapacity, input.rackMetres).size());
                auto fuelPositions = Positions(model_.FloorFuel());
                const auto hopperPositions = Positions(hopper);
                fuelPositions.insert(fuelPositions.end(), hopperPositions.begin(), hopperPositions.end());
                frc::SmartDashboard::PutNumber("FieldSimulation/SuccessfulScoreCount", model_.SuccessfulScores());
                frc::SmartDashboard::PutNumber("FieldSimulation/LaunchedFuelCount", model_.LaunchedFuel());
                frc::SmartDashboard::PutNumber("FieldSimulation/FuelProjectileCount", model_.Projectiles().size());
                frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelPositionsXYZ", fuelPositions);
                frc::SmartDashboard::PutNumberArray("FieldSimulation/HopperFuelPositionsXYZ", hopperPositions);
                frc::SmartDashboard::PutNumberArray("FieldSimulation/FuelProjectilePositionsXYZ", Positions(model_.Projectiles()));
        }
}
#endif
