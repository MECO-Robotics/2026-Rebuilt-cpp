#ifndef __FRC_ROBORIO__
#include "season/simulation/Simulation.h"
#include "season/simulation/FuelConfig.h"
#include "season/mechanisms/RobotGeometry.h"
#include "meco/sim/ParticleFieldContacts.h"
#include <networktables/NetworkTableInstance.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
#include <numbers>
namespace season
{
        namespace
        {
                constexpr double diameter = fuelconfig::kFuelDiameterMetres, radius = diameter / 2;
                const meco::sim::ParticleFieldContacts kFieldContacts{base::sim::fields::rebuilt2026::ParticleFieldColliders()};
                bool Outside(const FuelParticle &p)
                {
                        return p.position.x < 0 || p.position.x > base::sim::fields::rebuilt2026::kFieldLength ||
                               p.position.y < 0 || p.position.y > base::sim::fields::rebuilt2026::kFieldWidth;
                }
                void ReturnToField(FuelParticle &p)
                {
                        constexpr double inset = fuelconfig::kFieldReturnInsetMetres;
                        p.position = {std::clamp(p.position.x, inset, base::sim::fields::rebuilt2026::kFieldLength - inset),
                                      std::clamp(p.position.y, inset, base::sim::fields::rebuilt2026::kFieldWidth - inset), radius};
                        p.velocity = {};
                }
                frc::Translation3d Translation(FuelPoint p) { return {units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z}}; }
                FuelPoint Point3d(const frc::Translation3d &p) { return {p.X().value(), p.Y().value(), p.Z().value()}; }
                frc::Pose3d Pose3d(frc::Pose2d p) { return frc::Pose3d{p}; }
                FuelPoint World(FuelPoint local, const frc::Pose3d &robot) { return Point3d(robot.Translation() + Translation(local).RotateBy(robot.Rotation())); }
                FuelRobotState RobotState(const FuelInputs &in) { return in.physics.value_or(FuelRobotState{Pose3d(in.robot), {in.vx, in.vy, 0}, {0, 0, in.omega}, {}}); }
                // Four prescribed bumper boxes retain the open center of this robot.
                const std::vector<meco::sim::ContactBox> kBumpers = []
                {
                        constexpr double half = fuelconfig::kBumperHalfWidthMetres, depth = fuelconfig::kBumperDepthMetres, bottom = fuelconfig::kBumperBottomMetres, top = bottom + 2 * fuelconfig::kBumperHalfHeightMetres;
                        std::vector<meco::sim::ContactBox> boxes;
                        for (int side = 0; side < 4; ++side)
                        {
                                const bool alongX = side < 2;
                                const double sign = side % 2 ? 1 : -1;
                                boxes.push_back({{alongX ? sign * (half - depth / 2) : 0, alongX ? 0 : sign * (half - depth / 2), (bottom + top) / 2},
                                                 {alongX ? depth / 2 : half, alongX ? half : depth / 2, (top - bottom) / 2}});
                        }
                        return boxes;
                }();
                std::vector<double> Positions(const std::vector<FuelParticle> &particles)
                {
                        std::vector<double> values;
                        values.reserve(particles.size() * 3);
                        for (auto &p : particles)
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
                        for (auto &p : particles)
                        {
                                values.push_back(p.x);
                                values.push_back(p.y);
                                values.push_back(p.z);
                        }
                        return values;
                }
        }
        FuelModel::FuelModel(unsigned seed) : seed_(seed), random_(seed) { Reset(); }
        void FuelModel::Reset()
        {
                random_.seed(seed_);
                floor_.clear();
                airborne_.clear();
                stored_ = scored_ = launched_ = pending_ = burstIndex_ = 0;
                time_ = 0;
                lastBurst_ = nextShot_ = -1e9;
                lastHubActive_.reset();
                scoringGraceUntil_.reset();
        }
        void FuelModel::AddFloorFuel(frc::Translation2d point) { floor_.push_back({{point.X().value(), point.Y().value(), radius}, {}}); }
        void FuelModel::SetStoredFuel(int count) { stored_ = std::clamp(count, 0, fuelconfig::kSimulationCapacity); }
        double FuelModel::Random(double amplitude) { return std::uniform_real_distribution<double>{-amplitude, amplitude}(random_); }
        double FuelModel::LaunchPitch(double hood)
        {
                double deflection = std::clamp(hood * 2 * kPi + kShooterRestDeflectionDegrees * kPi / 180, fuelconfig::kMinHoodDeflectionDegrees * kPi / 180, fuelconfig::kMaxHoodDeflectionDegrees * kPi / 180);
                return kPi / 2 - deflection;
        }
        std::vector<FuelPoint> FuelModel::HopperCenters(int count, double extension, frc::Pose2d robot)
        {
                return HopperCenters3d(count, extension, Pose3d(robot));
        }
        std::vector<FuelPoint> FuelModel::HopperCenters3d(int count, double extension, const frc::Pose3d &robot)
        {
                std::vector<FuelPoint> centers;
                count = std::clamp(count, 0, fuelconfig::kSimulationCapacity);
                centers.reserve(static_cast<std::size_t>(count));
                double maxX = HopperPose(std::max(0., extension)).X().value();
                int zi = 0;
                for (double z = fuelconfig::kHopperFloorMetres + radius; z <= fuelconfig::kHopperCeilingMetres - radius + 1e-9; z += std::numbers::sqrt3 / 2 * diameter, ++zi)
                {
                        int xi = 0;
                        for (double x = fuelconfig::kHopperBackMetres + radius + (zi % 2 ? radius : 0); x <= maxX - radius + 1e-9; x += diameter, ++xi)
                                for (double y = -fuelconfig::kHopperHalfWidthMetres + radius + ((zi + xi) % 2 ? radius : 0); y <= fuelconfig::kHopperHalfWidthMetres - radius + 1e-9; y += diameter)
                                {
                                        if (static_cast<int>(centers.size()) == count)
                                                return centers;
                                        centers.push_back(World({x, y, z}, robot));
                                }
                }
                return centers;
        }
        void FuelModel::Launch(const FuelInputs &in, int index)
        {
                const auto robot = RobotState(in);
                double lateral = (index - fuelconfig::kBurstLateralCenter) * fuelconfig::kBurstLateralSpacingMetres;
                FuelPoint position = World({kShooterExitXMetres, lateral, kReleaseHeight}, robot.pose);
                double pitch = std::clamp(LaunchPitch(in.hoodRotations) + Random(fuelconfig::kLaunchPitchNoiseDegrees * kPi / 180), fuelconfig::kMinLaunchPitchDegrees * kPi / 180, fuelconfig::kMaxLaunchPitchDegrees * kPi / 180);
                double speed = std::abs(in.flywheelRps) * kProjectileSpeedPerRps * (1 + Random(fuelconfig::kLaunchSpeedNoiseFraction));
                const auto muzzleVelocity = Point3d(Translation({speed * std::cos(pitch), 0, speed * std::sin(pitch)}).RotateBy(robot.pose.Rotation()));
                const auto com = World(robot.centerOfMass, robot.pose);
                const double dx = position.x - com.x, dy = position.y - com.y, dz = position.z - com.z;
                const auto &w = robot.angularVelocity;
                airborne_.push_back({position, {muzzleVelocity.x + robot.velocity.x + w.y * dz - w.z * dy, muzzleVelocity.y + robot.velocity.y + w.z * dx - w.x * dz, muzzleVelocity.z + robot.velocity.z + w.x * dy - w.y * dx}});
                ++launched_;
        }
        void FuelModel::Step(double dt, const FuelInputs &in)
        {
                if (!std::isfinite(dt) || dt <= 0 || dt > fuelconfig::kMaxStepSeconds)
                        return;
                time_ += dt;
                // The field continues FUEL assessment for three seconds after deactivation,
                // including the AUTO and TELEOP ending gaps. Unknown state cannot establish
                // eligibility. Opening crossings approximate the physical sensor array.
                if (!in.hubActive)
                {
                        lastHubActive_.reset();
                        scoringGraceUntil_.reset();
                }
                else
                {
                        if (!*in.hubActive && lastHubActive_.value_or(false))
                                scoringGraceUntil_ = time_ + fuelconfig::kScoringGraceSeconds;
                        if (*in.hubActive)
                                scoringGraceUntil_.reset();
                        lastHubActive_ = in.hubActive;
                }
                const bool scoring = in.hubActive && (*in.hubActive || (scoringGraceUntil_ && time_ <= *scoringGraceUntil_));
                if (in.collecting && in.rackMetres > fuelconfig::kCollectionMinRackMetres && stored_ < fuelconfig::kSimulationCapacity)
                {
                        const auto robot = RobotState(in).pose;
                        for (auto it = floor_.begin(); it != floor_.end() && stored_ < fuelconfig::kSimulationCapacity;)
                        {
                                const auto local = (Translation(it->position) - robot.Translation()).RotateBy(-robot.Rotation());
                                double x = local.X().value(), y = local.Y().value();
                                if (x >= fuelconfig::kCollectionMouthXMetres && x <= fuelconfig::kCollectionMouthXMetres + fuelconfig::kCollectionMouthDepthMetres + radius && std::abs(y) <= fuelconfig::kCollectionHalfWidthMetres + radius)
                                {
                                        ++stored_;
                                        it = floor_.erase(it);
                                }
                                else
                                        ++it;
                        }
                }
                if (in.feeding && std::abs(in.flywheelRps) >= fuelconfig::kMinFeedFlywheelRps)
                {
                        if (!pending_ && time_ - lastBurst_ >= fuelconfig::kSimulationBurstPeriodSeconds)
                        {
                                pending_ = fuelconfig::kSimulationBurstSize;
                                burstIndex_ = 0;
                                lastBurst_ = (time_ - lastBurst_ > fuelconfig::kBurstResetPeriods * fuelconfig::kSimulationBurstPeriodSeconds) ? time_ : lastBurst_ + fuelconfig::kSimulationBurstPeriodSeconds;
                                nextShot_ = lastBurst_;
                        }
                        while (pending_ && time_ >= nextShot_)
                        {
                                if (!stored_)
                                {
                                        pending_ = 0;
                                        break;
                                }
                                --stored_;
                                Launch(in, burstIndex_++);
                                --pending_;
                                if (pending_)
                                        nextShot_ += std::max(fuelconfig::kMinLaunchGapSeconds, fuelconfig::kLaunchGapSeconds + Random(fuelconfig::kLaunchGapNoiseSeconds));
                        }
                }
                // Substeps resolve descending hub-height crossings even with coarse callers.
                int steps = std::max(1, static_cast<int>(std::ceil(dt / fuelconfig::kParticleStepSeconds)));
                double h = dt / steps;
                const double attenuation = std::exp(-fuelconfig::kFloorDampingPerSecond * h);
                const auto hub = HubPosition(in.alliance);
                const double height = kHubOpeningHeightMetres;
                for (int step = 0; step < steps; ++step)
                {
                        for (auto it = airborne_.begin(); it != airborne_.end();)
                        {
                                auto previous = it->position;
                                it->position.x += it->velocity.x * h;
                                it->position.y += it->velocity.y * h;
                                it->position.z += it->velocity.z * h - .5 * 9.80665 * h * h;
                                it->velocity.z -= 9.80665 * h;
                                bool crossing = previous.z >= height && it->position.z < height && it->velocity.z < 0;
                                double fraction = crossing ? (previous.z - height) / (previous.z - it->position.z) : 0;
                                double x = previous.x + (it->position.x - previous.x) * fraction, y = previous.y + (it->position.y - previous.y) * fraction;
                                if (Outside(*it))
                                {
                                        ReturnToField(*it);
                                        floor_.push_back(*it);
                                        it = airborne_.erase(it);
                                }
                                else if (crossing && std::abs(x - hub.X().value()) <= fuelconfig::kHubHalfWidthMetres && std::abs(y - hub.Y().value()) <= fuelconfig::kHubHalfWidthMetres)
                                {
                                        if (scoring)
                                                ++scored_;
                                        double direction = in.alliance == frc::DriverStation::Alliance::kBlue ? 1 : -1, angle = (direction > 0 ? 0 : kPi) + Random(fuelconfig::kReturnAngleNoiseDegrees * kPi / 180);
                                        floor_.push_back({{hub.X().value() + direction * fuelconfig::kScoredReturnOffsetMetres, hub.Y().value(), radius}, {fuelconfig::kScoredReturnSpeedMps * std::cos(angle), fuelconfig::kScoredReturnSpeedMps * std::sin(angle), 0}});
                                        it = airborne_.erase(it);
                                }
                                else if (it->position.z <= radius)
                                {
                                        floor_.push_back({{it->position.x, it->position.y, radius}, {it->velocity.x * fuelconfig::kGroundBounceScale, it->velocity.y * fuelconfig::kGroundBounceScale, 0}});
                                        it = airborne_.erase(it);
                                }
                                else
                                        ++it;
                        }
                        for (auto &p : floor_)
                        {
                                p.position.x += p.velocity.x * h;
                                p.position.y += p.velocity.y * h;
                                p.velocity.x *= attenuation;
                                p.velocity.y *= attenuation;
                        }
                        for (auto &p : floor_)
                                if (Outside(p))
                                        ReturnToField(p);
                        kFieldContacts.Resolve(floor_, radius, fuelconfig::kFieldRestitution);
                        kFieldContacts.Resolve(airborne_, radius, fuelconfig::kFieldRestitution);
                        meco::sim::ResolveParticleContacts(floor_, airborne_, RobotState(in), radius, fuelconfig::kFieldRestitution, fuelconfig::kRobotParticleFriction, kBumpers);
                        // Particle and bumper impulses can push fuel through a boundary.
                        for (auto &p : floor_)
                                if (Outside(p))
                                        ReturnToField(p);
                        for (auto it = airborne_.begin(); it != airborne_.end();)
                        {
                                if (Outside(*it))
                                {
                                        ReturnToField(*it);
                                        floor_.push_back(*it);
                                        it = airborne_.erase(it);
                                }
                                else
                                        ++it;
                        }
                        // A bumper or ball may lift resting fuel back into ballistic motion.
                        for (auto it = floor_.begin(); it != floor_.end();)
                        {
                                if (it->position.z > radius + 1e-6 || it->velocity.z > 1e-6)
                                {
                                        airborne_.push_back(*it);
                                        it = floor_.erase(it);
                                }
                                else
                                {
                                        it->position.z = radius;
                                        it->velocity.z = 0;
                                        ++it;
                                }
                        }
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
                const auto robotPose = RobotState(input).pose;
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
