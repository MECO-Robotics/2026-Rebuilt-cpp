#include "season/drive/OffensiveAutoDrive.h"
#include "season/mechanisms/Mechanisms.h"
#include "season/drive/ScoringPosition.h"
#include "season/runtime/SeasonPolicy.h"
#include "season/runtime/SeasonPolicy.h"
#include <base/AutoDrive.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

using namespace units::literals;
namespace season
{
  namespace
  {
    void Phase(const char *phase) { frc::SmartDashboard::PutString("AutoDrive/OffensePhase", phase); }

    struct FeedTracker
    {
      double start = 0, last = 0, elapsed = 0;
      bool active = false, requested = false;
      bool requestActive = false;
      double requestedBeforeTransport = 0, requestUpdatedAt = 0;
      double quietSince = 0;
      double quietTime = runtime::SeasonPolicy().behavior.emptyQuietSeconds;
      std::array<double, 3> rollerMean{};
      bool initialized = false, empty = false, latencyRecorded = false;
      std::optional<double> firstRequest;
      void Reset()
      {
        start = last = frc::Timer::GetFPGATimestamp().value();
        elapsed = 0;
        active = false;
        requested = false;
        requestActive = false;
        requestedBeforeTransport = 0;
        requestUpdatedAt = last;
        quietSince = last;
        initialized = empty = latencyRecorded = false;
        firstRequest.reset();
      }
      void ObserveRequest(bool ready)
      {
        const double now = frc::Timer::GetFPGATimestamp().value();
        if (requestActive && !active)
          requestedBeforeTransport += std::max(0., now - requestUpdatedAt);
        requestActive = ready;
        requestUpdatedAt = now;
      }
      void Update(bool ready, Mechanisms m)
      {
        const double now = frc::Timer::GetFPGATimestamp().value();
        if (active)
          elapsed += std::max(0., now - last);
        const double threshold = frc::Preferences::GetDouble("AutoDrive/EmptyShooterCurrentAmps", 0);
        const double noiseThreshold = frc::Preferences::GetDouble("AutoDrive/EmptyRollerNoiseAmps", 0);
        const std::array<double, 3> currents{m.conveyor.Inputs().current, m.bottomIndexer.Inputs().current,
                                             m.topIndexer.Inputs().current};
        const double shooter = m.shooter.Inputs().current;
        const bool valid = ready && threshold > 0 && noiseThreshold > 0 && std::isfinite(shooter) &&
                           m.shooter.Inputs().connected && m.conveyor.Inputs().connected && m.bottomIndexer.Inputs().connected &&
                           m.topIndexer.Inputs().connected && std::all_of(currents.begin(), currents.end(), [](double c)
                                                                          { return std::isfinite(c); });
        if (!valid)
        {
          quietSince = now;
          initialized = false;
        }
        else
        {
          if (!initialized)
          {
            rollerMean = currents;
            initialized = true;
            quietSince = now;
          }
          double noise = 0;
          for (size_t i = 0; i < currents.size(); ++i)
          {
            noise = std::max(noise, std::abs(currents[i] - rollerMean[i]));
            rollerMean[i] += offenseconfig::kEmptyCurrentFilterGain * (currents[i] - rollerMean[i]);
          }
          frc::SmartDashboard::PutNumber("AutoDrive/EmptyDetectorRollerNoiseAmps", noise);
          frc::SmartDashboard::PutNumber("AutoDrive/EmptyDetectorShooterAmps", shooter);
          if (shooter >= threshold || noise >= noiseThreshold)
            quietSince = now;
          if (now - quietSince >= quietTime)
            empty = true;
        }
        frc::SmartDashboard::PutNumber("AutoDrive/EmptyDetectorQuietSeconds", valid ? now - quietSince : 0.);
        frc::SmartDashboard::PutBoolean("AutoDrive/CurrentIndicatesEmpty", ready && empty);
        active = ready;
        last = now;
      }
    };
  } // namespace
  using offense::CollectionSpline;
  using offense::OffensiveMotionLimits;

  frc2::CommandPtr OffensiveAutoDrive(AutoDrive &perception, base::Drive &drive, Mechanisms m, FieldGeometry field,
                                      std::shared_ptr<OffensiveFuelEstimate> estimate)
  {
    // Each cycle chooses whether to collect, return and score, ferry, or stage.
    // The decision accounts for match timing and the estimated load; the named
    // actions below own navigation and mechanism commands for each phase.
    // Anonymous detections are only a rough pickup estimate: an observation
    // disappearing in the intake mouth may also be occluded. Never treat this
    // estimate as a measured hopper sensor or as a capacity guarantee.
    if (!estimate)
      estimate = std::make_shared<OffensiveFuelEstimate>();
    auto commitment = std::make_shared<OffensiveCommitment>(OffensiveCommitment::None);
    auto transportLatency = std::make_shared<double>(0.);
    auto ferryTransportLatency = std::make_shared<double>(0.);
    auto hubSetupLatency = std::make_shared<double>(0.);
    auto returnLatency = std::make_shared<double>(0.);
    auto stageLatency = std::make_shared<double>(0.);
    auto returnTravelSpeed = std::make_shared<double>(0.);
    auto stageTravelSpeed = std::make_shared<double>(0.);
    auto collectionStarted = std::make_shared<double>(0.);
    auto collectionStartCount = std::make_shared<int>(0);
    auto refillSeconds = std::make_shared<double>(0.);
    frc::Preferences::InitDouble("AutoDrive/EstimatedCapacity", 0);
    frc::Preferences::InitDouble("AutoDrive/FeedBallsPerSecond", 0);
    frc::Preferences::InitDouble("AutoDrive/EmptyShooterCurrentAmps", runtime::SeasonPolicy().behavior.emptyShooterCurrentAmps);
    frc::Preferences::InitDouble("AutoDrive/EmptyRollerNoiseAmps", runtime::SeasonPolicy().behavior.emptyRollerNoiseAmps);
    auto configuredCapacity = []
    {
      double capacity = frc::Preferences::GetDouble("AutoDrive/EstimatedCapacity", 0);
      if (capacity == 0)
        capacity = runtime::SeasonPolicy().behavior.defaultCapacity;
      return capacity;
    };
    auto feedDuration = []
    {
      const double duration = ConfiguredFeedDurationSeconds();
      frc::SmartDashboard::PutBoolean("AutoDrive/FeedConfigurationValid", duration > 0);
      frc::SmartDashboard::PutNumber("AutoDrive/FeedDurationSeconds", duration);
      return duration;
    };
    auto preparing = std::make_shared<bool>(false);
    auto prepareForInactive = [&perception, &drive, field, feedDuration, preparing]
    {
      const auto &match = perception.MatchState();
      if (!match.ownActive.has_value())
        return false;
      if (!*match.ownActive || match.phase == "endgame" || frc::DriverStation::IsAutonomousEnabled())
      {
        *preparing = false;
        frc::SmartDashboard::PutBoolean("AutoDrive/ReturnDeferredForShift", false);
        return false;
      }
      if (*preparing)
        return true;
      const auto alliance = frc::DriverStation::GetAlliance();
      const double duration = feedDuration();
      if (!alliance || !match.ownNextChange || !std::isfinite(*match.ownNextChange) || *match.ownNextChange < 0 ||
          !std::isfinite(duration) || duration <= 0)
        return false;
      const auto pose = drive.Pose();
      const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
      const double hubX = HubPosition(*alliance).X().value();
      const double laneY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
      const frc::Translation2d home{units::meter_t{hubX - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{laneY}};
      const frc::Translation2d neutral{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{laneY}};
      const bool canShootHere = InAllianceZone(*alliance, pose.Translation(), field) && RobotClearOfBumps(pose);
      const double toShot = canShootHere ? 0. : pose.Translation().Distance(neutral).value() + neutral.Distance(home).value();
      const double afterShot = canShootHere ? pose.Translation().Distance(home).value() + home.Distance(neutral).value()
                                            : home.Distance(neutral).value();
      const double speed = OffensiveMotionLimits(drive.Config()).maxSpeed;
      const double travel = speed > 0 ? (toShot + afterShot) / speed : 0.;
      frc::SmartDashboard::PutNumber("AutoDrive/NextActionTravelSeconds", travel);
      *preparing = speed > 0 && *match.ownNextChange < duration + travel;
      return *preparing;
    };
    auto preparingActive = std::make_shared<bool>(false);
    auto prepareForActive = [&perception, &drive, field, hubSetupLatency, returnTravelSpeed, stageTravelSpeed,
                             preparingActive]
    {
      const auto &match = perception.MatchState();
      if (!match.ownActive.has_value())
        return false;
      if (*match.ownActive || frc::DriverStation::IsAutonomousEnabled())
      {
        *preparingActive = false;
        return false;
      }
      if (*preparingActive)
        return true;
      const auto alliance = frc::DriverStation::GetAlliance();
      if (!alliance || !match.ownNextChange || !std::isfinite(*match.ownNextChange) || *match.ownNextChange < 0)
        return false;
      const auto pose = drive.Pose();
      const auto grid = LoadScoringGrid();
      const auto goal = grid ? CheapestHubScoringNode(*grid, pose, *alliance, field, runtime::SeasonPolicy().shot)
                             : std::nullopt;
      if (!goal)
        return false;
      const double distance = pose.Translation().Distance(goal->Translation()).value();
      const double observedSpeed = std::max(*stageTravelSpeed, *returnTravelSpeed);
      const double estimatedSpeed = std::clamp(observedSpeed > 0 ? observedSpeed : offenseconfig::kFallbackTravelSpeed, offenseconfig::kMinEstimatedTravelSpeed, offenseconfig::kMaxEstimatedTravelSpeed);
      const double travel = distance / estimatedSpeed;
      *preparingActive = std::isfinite(travel) && *match.ownNextChange < travel + *hubSetupLatency + offenseconfig::kHubPreparationMarginSeconds;
      return *preparingActive;
    };

    // Ferry only when the observed pickup rate leaves time for another full
    // load and a return before the HUB reactivates. Unknown rates or reversal
    // delays cannot establish that the additional cycle will fit.
    auto canFerry = [&perception, &drive, field, feedDuration, refillSeconds,
                     ferryTransportLatency, transportLatency, hubSetupLatency]
    {
      const auto &match = perception.MatchState();
      const auto alliance = frc::DriverStation::GetAlliance();
      const double duration = feedDuration();
      const double reversal = *ferryTransportLatency > 0 ? *ferryTransportLatency : *transportLatency;
      const double speed = OffensiveMotionLimits(drive.Config()).maxSpeed;
      if (!alliance || !match.ownActive || *match.ownActive || !match.ownNextChange ||
          !std::isfinite(*match.ownNextChange) || *match.ownNextChange <= 0 ||
          duration <= 0 || *refillSeconds <= 0 || reversal <= 0 || speed <= 0)
      {
        frc::SmartDashboard::PutBoolean("AutoDrive/FerryCycleFeasible", false);
        return false;
      }
      const auto pose = drive.Pose();
      const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
      const double hub = HubPosition(*alliance).X().value();
      const double y = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
      const frc::Translation2d neutral{units::meter_t{hub + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
      const frc::Translation2d home{units::meter_t{hub - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
      const double route = pose.Translation().Distance(neutral).value() + neutral.Distance(home).value();
      const double needed = duration + reversal + *refillSeconds + route / speed + *hubSetupLatency;
      frc::SmartDashboard::PutNumber("AutoDrive/FerryCycleSeconds", needed);
      const bool feasible = *match.ownNextChange > needed;
      frc::SmartDashboard::PutBoolean("AutoDrive/FerryCycleFeasible", feasible);
      return feasible;
    };

    auto avoidLateExit = std::make_shared<bool>(false);
    auto decision = [&perception, &drive, field, estimate, configuredCapacity, feedDuration, prepareForInactive, prepareForActive, canFerry, transportLatency, hubSetupLatency, avoidLateExit](bool expired,
                                                                                                                                                                                               std::optional<frc::Translation2d> retained = std::nullopt)
    {
      const auto alliance = frc::DriverStation::GetAlliance();
      const auto &match = perception.MatchState();
      if (alliance && prepareForInactive())
        return OffensiveDecision::Collect;
      if (alliance && prepareForActive())
        return estimate->Count() > 0 ? OffensiveDecision::Stage : OffensiveDecision::Collect;
      const bool windowClosing = alliance && !frc::DriverStation::IsAutonomousEnabled() &&
                                 match.ownActive.value_or(false) &&
                                 OffensiveScoringWindowClosing(drive.Pose(), *alliance, field, match.ownNextChange);
      // Reserve time in an inactive shift for aim, reversing the transport, and
      // feeding before the next active shift interrupts the ferry attempt.
      const bool ferryWindowClosing = !frc::DriverStation::IsAutonomousEnabled() &&
                                      match.ownActive.has_value() && !*match.ownActive &&
                                      OffensiveFerryWindowClosing(match.ownNextChange);
      // Do not spend most of a scoring window returning for a tiny, uncertain
      // estimate. Compare the estimated fraction of a hopper with the fraction
      // of the remaining window needed for travel and a configured full dump.
      bool worthwhileWindowReturn = true;
      bool worthwhileWindowFerry = true;
      const double capacity = configuredCapacity(), duration = feedDuration();
      const double speed = OffensiveMotionLimits(drive.Config()).maxSpeed;
      if (alliance && match.ownActive.has_value() && !*match.ownActive &&
          std::isfinite(capacity) && capacity > 0 && estimate->Count() >= capacity && !canFerry())
        return OffensiveDecision::Stage;
      if (windowClosing && alliance && match.ownNextChange && *match.ownNextChange > 0 &&
          std::isfinite(capacity) && capacity > 0 && std::isfinite(duration) && duration > 0 && speed > 0)
      {
        const auto pose = drive.Pose();
        const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
        const double hubX = HubPosition(*alliance).X().value();
        const double y = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        const frc::Translation2d home{units::meter_t{hubX - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const frc::Translation2d neutral{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const double route = InAllianceZone(*alliance, pose.Translation(), field) ? 0. : pose.Translation().Distance(neutral).value() + neutral.Distance(home).value();
        worthwhileWindowReturn = std::clamp(estimate->Count() / capacity, 0., 1.) >=
                                 (route / speed + duration) / *match.ownNextChange;
      }
      if (ferryWindowClosing && match.ownNextChange && *match.ownNextChange > 0 &&
          std::isfinite(capacity) && capacity > 0 && std::isfinite(duration) && duration > 0)
        worthwhileWindowFerry = std::clamp(estimate->Count() / capacity, 0., 1.) >=
                                duration / *match.ownNextChange;
      bool endMatchReturn = false;
      if (alliance && match.ownActive.value_or(false) && !frc::DriverStation::IsAutonomousEnabled() &&
          match.periodTime && std::isfinite(*match.periodTime) && *match.periodTime >= 0 &&
          estimate->Count() > 0 && std::isfinite(duration) && duration > 0 && speed > 0)
      {
        const auto pose = drive.Pose();
        const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
        const double hubX = HubPosition(*alliance).X().value();
        const double y = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        const frc::Translation2d home{units::meter_t{hubX - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const frc::Translation2d neutral{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const double route = InAllianceZone(*alliance, pose.Translation(), field) && RobotClearOfBumps(pose) ? 0. : pose.Translation().Distance(neutral).value() + neutral.Distance(home).value();
        endMatchReturn = *match.periodTime <= duration + *transportLatency + *hubSetupLatency + route / speed;
      }
      const auto choice = DecideOffense(perception.MatchState().ownActive, frc::DriverStation::GetAlliance(), drive.Pose(), field,
                                        perception.VisibleFuel(), estimate->Count(),
                                        capacity, false, false,
                                        frc::DriverStation::IsAutonomousEnabled(), perception.MatchState().periodTime,
                                        expired || endMatchReturn || *avoidLateExit || (estimate->Count() > 0 && ((windowClosing && worthwhileWindowReturn) || (ferryWindowClosing && worthwhileWindowFerry))), true, retained);
      if (choice == OffensiveDecision::Ferry && !canFerry())
        return std::isfinite(capacity) && capacity > 0 && estimate->Count() >= capacity
                   ? OffensiveDecision::Stage
                   : OffensiveDecision::Collect;
      return choice;
    };
    auto matchLost = std::make_shared<bool>(false);
    auto collect = [&perception, &drive, m, field, estimate, decision, matchLost, collectionStarted, collectionStartCount, refillSeconds, prepareForInactive, prepareForActive, canFerry, feedDuration, configuredCapacity, transportLatency, hubSetupLatency, avoidLateExit]()
    {
      auto selector = std::make_shared<OffensivePickupSelector>();
      // Load once per collection command, rather than reading the navgrid in
      // every target-supplier call on the scheduler thread.
      const auto collectionGrid = std::make_shared<std::optional<ScoringGrid>>(LoadScoringGrid());
      auto lastActive = std::make_shared<std::optional<bool>>();
      auto homeSearch = std::make_shared<bool>(false);
      struct Crossing
      {
        int stage = 0, inbound = 0;
        double laneY = 0;
      };
      auto crossing = std::make_shared<Crossing>();
      auto goal = [&perception, &drive, field, selector, collectionGrid, lastActive, homeSearch, estimate, crossing, prepareForInactive, prepareForActive, feedDuration, configuredCapacity, transportLatency, hubSetupLatency, avoidLateExit]() -> std::optional<frc::Pose2d>
      {
        const auto alliance = frc::DriverStation::GetAlliance();
        auto active = perception.MatchState().ownActive;
        if (!alliance || !active)
          return std::nullopt;
        if (prepareForInactive())
          active = false;
        if (*lastActive != active)
        {
          selector->Reset();
          *lastActive = active;
        }
        auto points = perception.VisibleFuel();
        const auto pose = drive.Pose();
        // Recover off a ramp before asking the pickup planner to move laterally.
        // A robot starting on a bump can otherwise remain pressed against it
        // through collection and repeated scoring attempts.
        if (!RobotClearOfBumps(pose))
        {
          const double hubX = HubPosition(*alliance).X().value();
          const double side = pose.X().value() >= hubX ? 1. : -1.;
          const frc::Translation2d clear{units::meter_t{hubX + side * offenseconfig::kHubCrossingOffset}, pose.Y()};
          frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalX", clear.X().value());
          frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalY", clear.Y().value());
          return frc::Pose2d{clear, pose.Rotation()};
        }
        if (!prepareForActive())
          *homeSearch = false;
        else if (!*homeSearch && InAllianceZone(*alliance, pose.Translation(), field))
        {
          *homeSearch = true;
          selector->Reset();
        }
        const bool preparingHome = *homeSearch;
        if (preparingHome)
          std::erase_if(points,
                        [alliance, field](const auto &p)
                        { return !InAllianceZone(*alliance, p.position, field); });
        else if (!*active)
          std::erase_if(points,
                        [alliance, field](const auto &p)
                        { return InAllianceZone(*alliance, p.position, field); });
        const auto speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(drive.Speeds(), pose.Rotation());
        const auto passable = [collectionGrid](frc::Translation2d p)
        {
          if (!std::isfinite(p.X().value()) || !std::isfinite(p.Y().value()))
            return false;
          if (!*collectionGrid)
            return true; // the pathfinder retains its own safety checks
          return base::IsNavigationGridPointFree(**collectionGrid, p);
        };
        auto selected = selector->Update(points, pose, speeds, frc::Timer::GetFPGATimestamp().value(), field, *active, passable);
        if (!selected)
        {
          // During an inbound shift transition, keep searching on the home side
          // rather than immediately undoing the crossing to chase neutral fuel.
          const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
          const double searchX = preparingHome ? HubPosition(*alliance).X().value() - direction * offenseconfig::kHubCrossingOffset : field.length / 2;
          const frc::Translation2d search{units::meter_t{searchX}, units::meter_t{field.width / 2}};
          selected = frc::Pose2d{search, (search - pose.Translation()).Angle()};
        }
        const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
        const auto &match = perception.MatchState();
        if (*active && match.phase != "endgame" && !frc::DriverStation::IsAutonomousEnabled() && estimate->Count() > 0 &&
            InAllianceZone(*alliance, pose.Translation(), field) && RobotClearOfBumps(pose) &&
            !InAllianceZone(*alliance, selected->Translation(), field) && match.ownNextChange &&
            std::isfinite(*match.ownNextChange) && *match.ownNextChange > 0)
        {
          const double speed = OffensiveMotionLimits(drive.Config()).maxSpeed;
          const double duration = feedDuration();
          const double capacity = configuredCapacity();
          const double y = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
          const double hub = HubPosition(*alliance).X().value();
          const frc::Translation2d home{units::meter_t{hub - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
          const frc::Translation2d neutral{units::meter_t{hub + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
          if (speed > 0 && std::isfinite(duration) && duration > 0 && std::isfinite(capacity) && capacity > 0 &&
              std::clamp(estimate->Count() / capacity, 0., 1.) >= duration / *match.ownNextChange)
          {
            const double afterShot = (pose.Translation().Distance(home).value() + home.Distance(neutral).value()) / speed;
            const double currentWork = duration + *transportLatency + *hubSetupLatency + afterShot;
            const double trip = (pose.Translation().Distance(selected->Translation()).value() +
                                 selected->Translation().Distance(neutral).value() + neutral.Distance(home).value()) /
                                speed;
            *avoidLateExit = *match.ownNextChange > currentWork &&
                             *match.ownNextChange <= currentWork + trip;
          }
        }
        if (preparingHome)
        {
          const double boundary = AllianceBoundary(*alliance, field);
          const double x = direction > 0 ? std::min(selected->X().value(), boundary)
                                         : std::max(selected->X().value(), boundary);
          *selected = frc::Pose2d{units::meter_t{x}, selected->Y(), selected->Rotation()};
        }
        const double hubX = HubPosition(*alliance).X().value();
        const double robotX = direction * (pose.X().value() - hubX);
        const double goalX = direction * (selected->X().value() - hubX);
        if (prepareForActive() && crossing->inbound == 0 && robotX > offenseconfig::kCrossingZoneOffset)
        {
          crossing->inbound = 1;
          crossing->stage = 0;
          crossing->laneY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        }
        if (crossing->inbound != 0)
        {
          auto waypoint = [&]
          { return frc::Translation2d{units::meter_t{hubX + direction * (crossing->inbound == 1 ? offenseconfig::kHubCrossingOffset : -offenseconfig::kHubCrossingOffset)}, units::meter_t{crossing->laneY}}; };
          if (pose.Translation().Distance(waypoint()).value() < offenseconfig::kCrossingWaypointTolerance)
            ++crossing->inbound;
          if (crossing->inbound > 2)
            crossing->inbound = 0;
          else
          {
            frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalX", waypoint().X().value());
            frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalY", waypoint().Y().value());
            return frc::Pose2d{waypoint(), pose.Rotation()};
          }
        }
        if (crossing->stage == 0 && robotX < -offenseconfig::kCrossingZoneOffset && goalX > offenseconfig::kCrossingZoneOffset)
        {
          crossing->stage = 1;
          crossing->laneY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        }
        if (crossing->stage != 0)
        {
          const frc::Translation2d waypoint{units::meter_t{hubX + direction * (crossing->stage == 1 ? -offenseconfig::kHubCrossingOffset : offenseconfig::kHubCrossingOffset)},
                                            units::meter_t{crossing->laneY}};
          if (pose.Translation().Distance(waypoint).value() < offenseconfig::kCrossingWaypointTolerance)
            ++crossing->stage;
          if (crossing->stage > 2)
            crossing->stage = 0;
          else
          {
            const frc::Translation2d next{units::meter_t{hubX + direction * (crossing->stage == 1 ? -offenseconfig::kHubCrossingOffset : offenseconfig::kHubCrossingOffset)},
                                          units::meter_t{crossing->laneY}};
            selected = frc::Pose2d{next, pose.Rotation()};
          }
        }
        frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalX", selected->X().value());
        frc::SmartDashboard::PutNumber("AutoDrive/CollectionGoalY", selected->Y().value());
        return selected;
      };
      base::AutoDriveTrackingConfig tracking;
      tracking.stabilitySeconds = offenseconfig::kTrackingStabilitySeconds;
      tracking.linearSettleSpeed = offenseconfig::kTrackingLinearSettleSpeed;
      tracking.angularSettleSpeed = offenseconfig::kTrackingAngularSettleSpeed;
      tracking.settleSamples = 1;
      auto collectionPath = frc2::cmd::Defer(
                                [&perception, &drive, field, estimate, configuredCapacity, prepareForInactive, prepareForActive,
                                 canFerry, goal, tracking]() -> frc2::CommandPtr
                                {
                                  const auto alliance = frc::DriverStation::GetAlliance();
                                  const auto active = perception.MatchState().ownActive;
                                  const auto grid = LoadScoringGrid();
                                  const double capacity = configuredCapacity();
                                  if (alliance && active && grid && std::isfinite(capacity) && capacity > 0 &&
                                      !prepareForInactive() && !prepareForActive() && RobotClearOfBumps(drive.Pose()))
                                  {
                                    const int remaining = std::max(0, static_cast<int>(std::ceil(capacity - estimate->Count())));
                                    const auto limits = OffensiveMotionLimits(drive.Config());
                                    auto path = CollectionSpline(drive.Pose(), perception.VisibleFuel(), *alliance, field, *grid,
                                                                 *active, !*active && canFerry(), remaining, limits);
                                    if (path)
                                    {
                                      frc::SmartDashboard::PutString("AutoDrive/CollectionPlan", "spline");
                                      frc::SmartDashboard::PutNumber("AutoDrive/CollectionChainWaypoints", path->getWaypoints().size() - 1);
                                      const auto config = drive.Config();
                                      const double maxTranslation = OffensiveMotionLimits(config).maxSpeed;
                                      auto controller = std::make_shared<pathplanner::PPHolonomicDriveController>(
                                          pathplanner::PIDConstants{config.autoTranslationKP, 0, 0},
                                          pathplanner::PIDConstants{config.autoRotationKP, 0, 0});
                                      return frc2::CommandPtr{std::make_unique<pathplanner::FollowPathCommand>(
                                          std::move(path), [&drive]
                                          { return drive.Pose(); }, [&drive]
                                          { return drive.Speeds(); },
                                          [&drive, maxTranslation, config](const frc::ChassisSpeeds &requested,
                                                                           const pathplanner::DriveFeedforwards &)
                                          {
                                            auto bounded = requested;
                                            const double magnitude = std::hypot(bounded.vx.value(), bounded.vy.value());
                                            if (magnitude > maxTranslation)
                                            {
                                              bounded.vx *= maxTranslation / magnitude;
                                              bounded.vy *= maxTranslation / magnitude;
                                            }
                                            bounded.omega = units::radians_per_second_t{
                                                std::clamp(bounded.omega.value(), -config.maxAngularRate, config.maxAngularRate)};
                                            drive.RunVelocity(bounded);
                                          },
                                          std::move(controller), pathplanner::RobotConfig::fromGUISettings(),
                                          []
                                          { return false; }, frc2::Requirements{&drive})};
                                    }
                                  }
                                  frc::SmartDashboard::PutString("AutoDrive/CollectionPlan", "pathfinder");
                                  frc::SmartDashboard::PutNumber("AutoDrive/CollectionChainWaypoints", 0);
                                  return base::AutoDriveTrackTarget(drive, goal, OffensiveMotionLimits(drive.Config()), tracking);
                                },
                                {&drive})
                                .Repeatedly();
      auto observePickups = frc2::cmd::Run(
          [&perception, &drive, estimate, collectionStarted, collectionStartCount, refillSeconds, configuredCapacity]
          {
            estimate->Observe(perception.VisibleFuel(), drive.Pose());
            const double elapsed = frc::Timer::GetFPGATimestamp().value() - *collectionStarted;
            const double capacity = configuredCapacity();
            const double pickups = std::min(capacity, static_cast<double>(estimate->Count() - *collectionStartCount));
            if (elapsed >= offenseconfig::kCollectionSpeedSampleSeconds && std::isfinite(capacity) && capacity > 0 && pickups > 0)
              *refillSeconds = capacity * elapsed / pickups;
            frc::SmartDashboard::PutNumber("AutoDrive/EstimatedFuel", estimate->Count());
            frc::SmartDashboard::PutNumber("AutoDrive/EstimatedRefillSeconds", *refillSeconds);
          });
      // The timer bounds this attempt. Driving, intake, and observation run
      // together until the timer expires or the next decision ends collection.
      return frc2::cmd::Deadline(frc2::cmd::Wait(offenseconfig::kCollectionTimeout),
                                 std::move(collectionPath),
                                 Acquire(m), std::move(observePickups))
          .Until(
              [decision, matchLost, selector]
              {
                auto next = decision(false, selector->Target());
                if (next == OffensiveDecision::Wait)
                  *matchLost = true;
                return next != OffensiveDecision::Collect;
              })
          .BeforeStarting(
              [selector, lastActive, homeSearch, estimate, matchLost, collectionStarted, collectionStartCount, crossing, avoidLateExit]
              {
                selector->Reset();
                lastActive->reset();
                *homeSearch = false;
                *avoidLateExit = false;
                crossing->stage = 0;
                crossing->inbound = 0;
                estimate->StartCollection();
                *matchLost = false;
                *collectionStarted = frc::Timer::GetFPGATimestamp().value();
                *collectionStartCount = estimate->Count();
                Phase("collect");
              });
    };
    auto scoringGoal = std::make_shared<std::optional<frc::Pose2d>>();
    auto returnHome = [&perception, &drive, field, scoringGoal, prepareForInactive, returnLatency, stageLatency,
                       returnTravelSpeed, stageTravelSpeed](bool staging = false)
    {
      auto start = std::make_shared<double>(0.);
      auto origin = std::make_shared<frc::Pose2d>();
      auto invalid = [&perception, prepareForInactive, staging]
      {
        const auto active = perception.MatchState().ownActive;
        return !active || !frc::DriverStation::GetAlliance().has_value() ||
               (staging ? *active : !*active || prepareForInactive());
      };
      // The navgrid can mark the depot occupied even when the robot has driven
      // in to collect. Move straight out before asking it for a shooting node;
      // turning in place here can pin the bumper against a depot rail.
      auto clearDepot = frc2::cmd::RunEnd([&drive]
                                          {
      const auto alliance = frc::DriverStation::GetAlliance();
      if (!alliance) { drive.Stop(); return; }
      const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
      drive.RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          units::meters_per_second_t{direction * offenseconfig::kClearSpeedMps}, 0_mps, 0_rad_per_s, drive.Pose().Rotation())); }, [&drive]
                                          { drive.Stop(); }, {&drive})
                            .Until([&drive, field]
                                   {
          const auto alliance = frc::DriverStation::GetAlliance();
          if (!alliance) return true;
          auto pose = drive.Pose();
          // Continue a little beyond the rotation envelope to account for
          // stopping distance before selecting the current point as a goal.
          const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
          pose = frc::Pose2d{units::meter_t{pose.X().value() - direction * offenseconfig::kRetreatBufferMetres}, pose.Y(), pose.Rotation()};
          return RobotClearOfDepot(pose, *alliance, field); })
                            .WithTimeout(offenseconfig::kRetreatTimeout)
                            .OnlyIf([&drive, field]
                                    {
          const auto alliance = frc::DriverStation::GetAlliance();
          return alliance && !RobotClearOfDepot(drive.Pose(), *alliance, field); });
      // The static grid may mark a pose close to a bump as blocked. Retreat in
      // the field's X direction before planning so a conservative clearance
      // boundary cannot leave a level robot repeatedly without a scoring goal.
      auto retreatDirection = [&drive]
      {
        const double x = drive.Pose().X().value();
        const double blue = HubPosition(frc::DriverStation::Alliance::kBlue).X().value();
        const double red = HubPosition(frc::DriverStation::Alliance::kRed).X().value();
        const double hub = std::abs(x - blue) < std::abs(x - red) ? blue : red;
        return x >= hub ? 1. : -1.;
      };
      auto clearBump = frc2::cmd::RunEnd([&drive, retreatDirection]
                                         { drive.RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                               units::meters_per_second_t{retreatDirection() * offenseconfig::kRetreatSpeedMps}, 0_mps, 0_rad_per_s, drive.Pose().Rotation())); }, [&drive]
                                         { drive.Stop(); }, {&drive})
                           .Until([&drive, retreatDirection]
                                  {
          const auto pose = drive.Pose();
          const auto buffered = frc::Pose2d{units::meter_t{pose.X().value() - retreatDirection() * offenseconfig::kRetreatBufferMetres},
                                             pose.Y(), pose.Rotation()};
          return RobotClearOfBumps(buffered); })
                           .WithTimeout(offenseconfig::kRetreatTimeout)
                           .OnlyIf([&drive]
                                   { return !RobotClearOfBumps(drive.Pose()); });
      auto navigate = frc2::cmd::Defer([&drive, field, scoringGoal, invalid, staging]() -> frc2::CommandPtr
                                       {
      scoringGoal->reset();
      const auto alliance = frc::DriverStation::GetAlliance();
      const auto grid = LoadScoringGrid();
      if (!alliance || !grid || invalid()) return frc2::cmd::None();
      const bool blue = *alliance == frc::DriverStation::Alliance::kBlue;
      const double direction = blue ? 1. : -1.;
      const double hubX = HubPosition(*alliance).X().value();
      const auto pose = drive.Pose();
      // Approaching the HUB from midfield through a bump can stall the robot.
      // Use the closer trench for this cross-field return; the two waypoints
      // keep the crossing inside the lane rather than cutting the bump corner.
      const double neutralOffset = direction * (pose.X().value() - hubX);
      if (neutralOffset > offenseconfig::kCrossingZoneOffset || (neutralOffset > 0 && !RobotClearOfBumps(pose)))
      {
        const double laneY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        const frc::Pose2d entry{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{laneY}, pose.Rotation()};
        const frc::Pose2d exit{units::meter_t{hubX - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{laneY}, pose.Rotation()};
        auto travel = [&drive](frc::Pose2d waypoint) {
          return base::AutoDriveToPose(drive, [waypoint] { return std::optional{waypoint}; }, OffensiveMotionLimits(drive.Config()))
              .Until([&drive, waypoint] { return drive.Pose().Translation().Distance(waypoint.Translation()).value() < offenseconfig::kEntryToleranceMetres; })
              .WithTimeout(offenseconfig::kEntryTimeout);
        };
        auto finish = frc2::cmd::Defer([&drive, field, scoringGoal, alliance, grid]() -> frc2::CommandPtr {
          *scoringGoal = CheapestHubScoringNode(*grid, drive.Pose(), *alliance, field, runtime::SeasonPolicy().shot);
          if (!*scoringGoal) return frc2::cmd::None();
          frc::SmartDashboard::PutNumber("AutoDrive/ScoringGoalX", (*scoringGoal)->X().value());
          frc::SmartDashboard::PutNumber("AutoDrive/ScoringGoalY", (*scoringGoal)->Y().value());
          return base::AutoDriveToPose(drive, [scoringGoal] { return *scoringGoal; }, OffensiveMotionLimits(drive.Config())).WithTimeout(offenseconfig::kScoringApproachTimeout);
        }, {&drive});
        // If collection left us on the neutral-facing ramp, back down it
        // before moving sideways into the trench lane. A direct path toward
        // a scoring cell otherwise keeps pushing the chassis into the ramp.
        const frc::Pose2d retreat{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, pose.Y(), pose.Rotation()};
        auto clearRamp = travel(retreat).OnlyIf([pose] { return !RobotClearOfBumps(pose); });
        return std::move(clearRamp).AndThen(travel(entry))
            .AndThen(travel(exit).AndThen(std::move(finish)).OnlyIf([&drive, entry] {
              return drive.Pose().Translation().Distance(entry.Translation()).value() < offenseconfig::kEntryHandoffToleranceMetres;
            }))
            .WithTimeout(offenseconfig::kTrenchReturnTimeout).Until(invalid);
      }
      *scoringGoal = CheapestHubScoringNode(*grid, pose, *alliance, field, runtime::SeasonPolicy().shot);
      if (!*scoringGoal) return frc2::cmd::None();
      frc::SmartDashboard::PutNumber("AutoDrive/ScoringGoalX", (*scoringGoal)->X().value());
      frc::SmartDashboard::PutNumber("AutoDrive/ScoringGoalY", (*scoringGoal)->Y().value());
      if (pose.Translation().Distance((*scoringGoal)->Translation()).value() < offenseconfig::kGoalPositionEqualityMetres &&
          (!staging || std::abs(std::remainder((pose.Rotation() - (*scoringGoal)->Rotation()).Radians().value(),
                                               2 * std::numbers::pi)) < offenseconfig::kGoalHeadingEqualityRadians))
        return frc2::cmd::None();
      return base::AutoDriveToPose(drive, [scoringGoal] { return *scoringGoal; }, OffensiveMotionLimits(drive.Config())).WithTimeout(offenseconfig::kDirectReturnTimeout).Until(invalid); }, {&drive});
      return std::move(clearDepot).AndThen(std::move(clearBump)).AndThen(std::move(navigate)).Until(invalid).BeforeStarting([start, origin, staging, &drive]
                                                                                                                            {
      *start = frc::Timer::GetFPGATimestamp().value();
      *origin = drive.Pose();
      Phase(staging ? "stage-for-hub" : "return"); })
          .FinallyDo([&drive, scoringGoal, start, origin, returnLatency, stageLatency, returnTravelSpeed, stageTravelSpeed, staging](bool interrupted)
                     {
      if (!interrupted && *scoringGoal &&
          drive.Pose().Translation().Distance((*scoringGoal)->Translation()).value() < offenseconfig::kGoalArrivalMetres)
      {
        const double elapsed = std::max(0., frc::Timer::GetFPGATimestamp().value() - *start);
        const double distance = origin->Translation().Distance((*scoringGoal)->Translation()).value();
        const double speed = elapsed > 0 ? distance / elapsed : 0.;
        if (staging)
        {
          *stageLatency = elapsed;
          if (speed > 0) *stageTravelSpeed = speed;
          frc::SmartDashboard::PutNumber("AutoDrive/ObservedStageSeconds", elapsed);
        }
        else
        {
          *returnLatency = elapsed;
          if (speed > 0) *returnTravelSpeed = speed;
          frc::SmartDashboard::PutNumber("AutoDrive/ObservedReturnSeconds", elapsed);
        }
      } });
    };
    auto shot = [&perception, &drive, m, field, estimate, commitment, feedDuration, configuredCapacity, prepareForInactive, transportLatency, hubSetupLatency]()
    {
      CoordinatedShotOptions options;
      options.ownDrive = true;
      options.autonomous = true;
      options.agitate = true;
      options.shotCalibration = runtime::SeasonPolicy().shot;
      options.readyTimeoutSeconds = shotconfig::kShotReadyTimeoutSeconds;
      auto requestedFeed = std::make_shared<double>(0.);
      options.feedDurationSeconds = [requestedFeed, feedDuration]
      {
        return *requestedFeed = feedDuration();
      };
      options.totalTimeoutSeconds = runtime::SeasonPolicy().behavior.shotTimeoutSeconds;
      options.poseReady = [&perception, &drive, field]
      {
        const auto alliance = frc::DriverStation::GetAlliance();
        return perception.MatchState().ownActive.value_or(false) && alliance &&
               HubScoringPositionValid(drive.Pose(), *alliance, field, runtime::SeasonPolicy().shot);
      };
      auto fed = std::make_shared<FeedTracker>();
      options.feedFeedback = [fed, m, transportLatency](bool active)
      {
        if (active && fed->firstRequest && !fed->latencyRecorded)
        {
          *transportLatency = std::max(0., frc::Timer::GetFPGATimestamp().value() - *fed->firstRequest);
          fed->latencyRecorded = true;
          frc::SmartDashboard::PutNumber("AutoDrive/ObservedTransportLatencySeconds", *transportLatency);
        }
        fed->Update(active, m);
      };
      options.readyFeedback = [fed, hubSetupLatency](bool ready)
      {
        if (ready && !fed->firstRequest)
        {
          fed->firstRequest = frc::Timer::GetFPGATimestamp().value();
          *hubSetupLatency = std::max(0., *fed->firstRequest - fed->start);
          frc::SmartDashboard::PutNumber("AutoDrive/ObservedHubSetupSeconds", *hubSetupLatency);
        }
        fed->requested |= ready;
      };
      options.feedComplete = [fed]
      { return fed->empty; };
      auto target = []
      {
        auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
        return ShotTarget{ShotMode::Hub, HubPosition(alliance)};
      };
      return CoordinatedShot(m, drive, field, target, options)
          .Until([&perception, &drive, field, fed, prepareForInactive]
                 {
          const auto alliance = frc::DriverStation::GetAlliance();
          return !perception.MatchState().ownActive.value_or(false) || !alliance ||
                 !HubScoringPositionValid(drive.Pose(), *alliance, field, runtime::SeasonPolicy().shot) ||
                 (prepareForInactive() && !fed->requested); })
          .BeforeStarting(
              [fed]
              {
                fed->Reset();
                Phase("score");
              })
          .FinallyDo(
              [estimate, fed, commitment, requestedFeed, configuredCapacity](bool interrupted)
              {
                const bool complete = !interrupted && *requestedFeed > 0 && (fed->elapsed >= *requestedFeed || fed->empty);
                if (complete)
                  estimate->CompleteFeed(true);
                else
                  estimate->AccountPartialFeed(fed->elapsed, *requestedFeed, configuredCapacity());
                frc::SmartDashboard::PutNumber("AutoDrive/EstimatedFuel", estimate->Count());
                *commitment = ReconcileOffensiveCommitment(*commitment, std::nullopt, complete);
              });
    };
    auto ferry = [&perception, &drive, m, field, estimate, commitment, feedDuration, configuredCapacity, prepareForActive, canFerry, ferryTransportLatency]()
    {
      CoordinatedShotOptions options;
      options.ownDrive = true;
      options.autonomous = true;
      options.agitate = true;
      options.shotCalibration = runtime::SeasonPolicy().shot;
      // Drift toward the neutral-side trench approach while aiming, so ferrying
      // also shortens the return to the alliance zone for the next active shift.
      options.ferryTranslation = [&drive, field]
      {
        const auto alliance = frc::DriverStation::GetAlliance();
        if (!alliance)
          return frc::Translation2d{};
        const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
        const auto pose = drive.Pose();
        const double targetX = HubPosition(*alliance).X().value() + direction * offenseconfig::kHubCrossingOffset;
        const double targetY = pose.Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        const double dx = targetX - pose.X().value(), dy = targetY - pose.Y().value();
        const double distance = std::hypot(dx, dy);
        const double scale = distance > offenseconfig::kFerryTranslationSpeedMps ? offenseconfig::kFerryTranslationSpeedMps / distance : 1.;
        return frc::Translation2d{units::meter_t{dx * scale}, units::meter_t{dy * scale}};
      };
      options.readyTimeoutSeconds = shotconfig::kShotReadyTimeoutSeconds;
      auto requestedFeed = std::make_shared<double>(0.);
      options.feedDurationSeconds = [requestedFeed, feedDuration]
      {
        return *requestedFeed = feedDuration();
      };
      options.totalTimeoutSeconds = runtime::SeasonPolicy().behavior.shotTimeoutSeconds;
      auto fed = std::make_shared<FeedTracker>();
      options.feedFeedback = [fed, m, ferryTransportLatency](bool active)
      {
        if (active && fed->firstRequest && !fed->latencyRecorded)
        {
          *ferryTransportLatency = fed->requestedBeforeTransport;
          fed->latencyRecorded = true;
          frc::SmartDashboard::PutNumber("AutoDrive/ObservedFerryTransportLatencySeconds", *ferryTransportLatency);
        }
        fed->Update(active, m);
      };
      options.readyFeedback = [fed](bool ready)
      {
        fed->ObserveRequest(ready);
        if (ready && !fed->firstRequest)
          fed->firstRequest = frc::Timer::GetFPGATimestamp().value();
        fed->requested |= ready;
      };
      options.feedComplete = [fed]
      { return fed->empty; };
      // As with HUB shots, the whole robot must be clear of the bump before
      // firing: a valid ballistic solution does not account for chassis tilt.
      options.poseReady = [&drive]
      { return RobotClearOfBumps(drive.Pose()); };
      auto target = [&drive, field]
      {
        auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
        return FerryTarget(alliance, drive.Pose(), field);
      };
      // If the current position cannot safely ferry, use a known neutral-side
      // lane and the normal obstacle-aware pathfinder before trying to shoot.
      auto positionForFerry = [&drive, field]() -> std::optional<frc::Pose2d>
      {
        const auto alliance = frc::DriverStation::GetAlliance();
        if (!alliance)
          return std::nullopt;
        const auto calibration = runtime::SeasonPolicy().shot;
        auto aimed = [&](frc::Pose2d pose)
        {
          for (int i = 0; i < 3; ++i)
          {
            const auto solution = CalculateShot(pose, FerryTarget(*alliance, pose, field), field, calibration);
            pose = frc::Pose2d{pose.Translation(), frc::Rotation2d{units::radian_t{solution.headingRadians}}};
          }
          return pose;
        };
        const auto current = aimed(drive.Pose());
        if (RobotClearOfBumps(current) &&
            CalculateShot(current, FerryTarget(*alliance, current, field), field, calibration).calibrated)
          return current;
        const double x = HubPosition(*alliance).X().value() +
                         (*alliance == frc::DriverStation::Alliance::kBlue ? offenseconfig::kHubCrossingOffset : -offenseconfig::kHubCrossingOffset);
        std::optional<frc::Pose2d> best;
        for (double y : {offenseconfig::kTrenchLaneOffset, field.width - offenseconfig::kTrenchLaneOffset})
        {
          const auto candidate = aimed(frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{}});
          if (!RobotClearOfBumps(candidate) ||
              !CalculateShot(candidate, FerryTarget(*alliance, candidate, field), field, calibration).calibrated)
            continue;
          if (!best || drive.Pose().Translation().Distance(candidate.Translation()) <
                           drive.Pose().Translation().Distance(best->Translation()))
            best = candidate;
        }
        return best;
      };
      auto ferryGoal = std::make_shared<std::optional<frc::Pose2d>>();
      auto reposition = frc2::cmd::Defer([&drive, positionForFerry, ferryGoal]() -> frc2::CommandPtr
                                         {
      *ferryGoal = positionForFerry();
      if (!*ferryGoal) return frc2::cmd::None();
      if (drive.Pose().Translation().Distance((*ferryGoal)->Translation()).value() < offenseconfig::kFerryGoalEqualityMetres)
        return frc2::cmd::None();
      const auto alliance = frc::DriverStation::GetAlliance();
      if (!alliance) return frc2::cmd::None();
      const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
      const double hubX = HubPosition(*alliance).X().value();
      if (!RobotClearOfBumps(drive.Pose()))
      {
        const frc::Pose2d retreat{units::meter_t{hubX + direction * offenseconfig::kHubCrossingOffset}, drive.Pose().Y(), drive.Pose().Rotation()};
        return base::AutoDriveToPose(drive, [retreat] { return std::optional{retreat}; }, OffensiveMotionLimits(drive.Config()))
            .Until([&drive] { return RobotClearOfBumps(drive.Pose()); })
            .WithTimeout(offenseconfig::kScoringApproachTimeout)
            .AndThen(base::AutoDriveToPose(drive, [ferryGoal] { return *ferryGoal; }, OffensiveMotionLimits(drive.Config())).WithTimeout(offenseconfig::kFerryRepositionTimeout)
                         .OnlyIf([&drive] { return RobotClearOfBumps(drive.Pose()); }));
      }
      // A robot alongside or on a bump must reach the alliance-side lane
      // before crossing. A direct diagonal to midfield can push it into the
      // bump or trench support and leave it unable to reach a ferry position.
      if (direction * (drive.Pose().X().value() - hubX) < offenseconfig::kCrossingZoneOffset)
      {
        const frc::Pose2d entry{units::meter_t{hubX - direction * offenseconfig::kHubCrossingOffset}, (*ferryGoal)->Y(), drive.Pose().Rotation()};
        return base::AutoDriveToPose(drive, [entry] { return std::optional{entry}; }, OffensiveMotionLimits(drive.Config()))
            .Until([&drive, entry] { return drive.Pose().Translation().Distance(entry.Translation()).value() < offenseconfig::kEntryToleranceMetres; })
            .WithTimeout(offenseconfig::kFerryEntryTimeout)
            .AndThen(base::AutoDriveToPose(drive, [ferryGoal] { return *ferryGoal; }, OffensiveMotionLimits(drive.Config())).WithTimeout(offenseconfig::kFerryApproachTimeout)
                         .OnlyIf([&drive, entry] {
                           return drive.Pose().Translation().Distance(entry.Translation()).value() < offenseconfig::kEntryHandoffToleranceMetres;
                         }));
      }
      return base::AutoDriveToPose(drive, [ferryGoal] { return *ferryGoal; }, OffensiveMotionLimits(drive.Config())).WithTimeout(offenseconfig::kFerryRepositionTimeout); }, {&drive});
      return std::move(reposition).AndThen(CoordinatedShot(m, drive, field, target, options)).Until([&perception, fed, prepareForActive, canFerry]
                                                                                                    { return !perception.MatchState().ownActive.has_value() || *perception.MatchState().ownActive ||
                                                                                                             ((prepareForActive() || !canFerry()) && fed->elapsed <= 0 && !fed->active); })
          .BeforeStarting([fed]
                          {
              fed->Reset();
              Phase("ferry"); })
          .FinallyDo([estimate, fed, commitment, requestedFeed, configuredCapacity, canFerry, ferryTransportLatency](bool interrupted)
                     {
              // A canceled reversal is a lower bound on the required motor
              // turnaround time; do not fall back to an older, faster hub shot.
              if (!fed->latencyRecorded && fed->requestedBeforeTransport > *ferryTransportLatency)
              {
                *ferryTransportLatency = fed->requestedBeforeTransport;
                frc::SmartDashboard::PutNumber("AutoDrive/ObservedFerryTransportLatencySeconds", *ferryTransportLatency);
              }
              const bool complete = !interrupted && *requestedFeed > 0 && (fed->elapsed >= *requestedFeed || fed->empty);
              if (complete)
                estimate->CompleteFeed(true);
              else
                estimate->AccountPartialFeed(fed->elapsed, *requestedFeed, configuredCapacity());
              frc::SmartDashboard::PutNumber("AutoDrive/EstimatedFuel", estimate->Count());
              *commitment = ReconcileOffensiveCommitment(*commitment, std::nullopt, complete);
              if (fed->elapsed <= 0 && !fed->active && !canFerry()) *commitment = OffensiveCommitment::None; });
    };
    auto waitForKnown = [&perception]()
    {
      return frc2::cmd::WaitUntil(
                 [&perception]
                 {
                   return frc::DriverStation::GetAlliance().has_value() && perception.MatchState().ownActive.has_value();
                 })
          .BeforeStarting([]
                          { Phase("wait-for-match"); });
    };
    auto mayShoot = [&perception, &drive, field, scoringGoal, prepareForInactive]
    {
      const auto alliance = frc::DriverStation::GetAlliance();
      return alliance && !prepareForInactive() && perception.MatchState().ownActive.value_or(false) && *scoringGoal &&
             HubScoringPositionValid(drive.Pose(), *alliance, field, runtime::SeasonPolicy().shot) &&
             drive.Pose().Translation().Distance((*scoringGoal)->Translation()).value() < offenseconfig::kGoalArrivalMetres;
    };
    // Preserve a committed attempt through navigation failure or an incomplete
    // feed. A hub-state change releases it; unknown match data preserves it but
    // waits. The estimated load is not treated as proof of an empty hopper.
    auto next = [decision, matchLost, collectionStarted, commitment, estimate, prepareForInactive, prepareForActive,
                 &perception, &drive, field, feedDuration, transportLatency, hubSetupLatency, returnLatency, preparing]
    {
      if (*matchLost)
        return OffensiveDecision::Wait;
      if (prepareForInactive())
      {
        *commitment = OffensiveCommitment::None;
        return OffensiveDecision::Collect;
      }
      if (prepareForActive())
      {
        *commitment = OffensiveCommitment::None;
        return estimate->Count() > 0 ? OffensiveDecision::Stage : OffensiveDecision::Collect;
      }
      if (*commitment == OffensiveCommitment::Score)
        return OffensiveDecision::Score;
      if (*commitment == OffensiveCommitment::Ferry)
        return OffensiveDecision::Ferry;
      const auto choice = decision(frc::Timer::GetFPGATimestamp().value() - *collectionStarted >= offenseconfig::kCollectionDecisionSeconds);
      const auto alliance = frc::DriverStation::GetAlliance();
      const auto &match = perception.MatchState();
      if (choice == OffensiveDecision::Score && alliance && match.phase != "endgame" &&
          !frc::DriverStation::IsAutonomousEnabled() && match.ownNextChange && std::isfinite(*match.ownNextChange) && *match.ownNextChange >= 0 &&
          !InAllianceZone(*alliance, drive.Pose().Translation(), field) && *returnLatency > 0)
      {
        const double direction = *alliance == frc::DriverStation::Alliance::kBlue ? 1. : -1.;
        const double hub = HubPosition(*alliance).X().value();
        const double y = drive.Pose().Y().value() < field.width / 2 ? offenseconfig::kTrenchLaneOffset : field.width - offenseconfig::kTrenchLaneOffset;
        const frc::Translation2d home{units::meter_t{hub - direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const frc::Translation2d neutral{units::meter_t{hub + direction * offenseconfig::kHubCrossingOffset}, units::meter_t{y}};
        const double speed = OffensiveMotionLimits(drive.Config()).maxSpeed;
        const double duration = feedDuration();
        if (speed > 0 && duration > 0 && *match.ownNextChange <= *returnLatency + duration + *hubSetupLatency + *transportLatency + home.Distance(neutral).value() / speed)
        {
          *preparing = true;
          frc::SmartDashboard::PutBoolean("AutoDrive/ReturnDeferredForShift", true);
          return OffensiveDecision::Collect;
        }
      }
      return choice;
    };
    auto reconcile = frc2::cmd::RunOnce(
        [&perception, commitment]
        { *commitment = ReconcileOffensiveCommitment(*commitment, perception.MatchState().ownActive, false); });
    auto record = frc2::cmd::RunOnce(
        [next, commitment]
        {
          auto choice = next();
          if (choice == OffensiveDecision::Score)
            *commitment = OffensiveCommitment::Score;
          if (choice == OffensiveDecision::Ferry)
            *commitment = OffensiveCommitment::Ferry;
        });
    auto warmShooter = [&drive, m]
    {
      return frc2::cmd::RunEnd(
          [&drive, m]
          {
            const auto alliance = frc::DriverStation::GetAlliance();
            if (!alliance)
            {
              m.shooter.Stop();
              m.hood.Stop();
              return;
            }
            const auto solution = CalculateShot(drive.Pose(), HubPosition(*alliance), runtime::SeasonPolicy().shot);
            m.shooter.SetVelocity(ClampCalibrationFlywheel(solution.flywheelRps));
            m.hood.SetPosition(ClampCalibrationHood(solution.hoodRotations));
          },
          [m]
          { m.shooter.Stop(); m.hood.Stop(); }, {&m.shooter, &m.hood});
    };
    auto cycle =
        frc2::cmd::Sequence(
            waitForKnown(), std::move(reconcile),
            frc2::cmd::Either(collect(), frc2::cmd::None(),
                              [commitment]
                              { return *commitment == OffensiveCommitment::None; }),
            std::move(record),
            frc2::cmd::Either(
                frc2::cmd::Sequence(frc2::cmd::Deadline(returnHome(), warmShooter()),
                                    frc2::cmd::Either(shot(), frc2::cmd::None(), mayShoot)),
                frc2::cmd::Either(ferry(),
                                  frc2::cmd::Either(
                                      frc2::cmd::Deadline(
                                          frc2::cmd::WaitUntil([&perception]
                                                               { return !perception.MatchState().ownActive.has_value() || *perception.MatchState().ownActive; }),
                                          frc2::cmd::Sequence(
                                              frc2::cmd::Deadline(
                                                  frc2::cmd::WaitUntil([prepareForActive]
                                                                       { return prepareForActive(); })
                                                      .BeforeStarting([]
                                                                      { Phase("stage-wait"); }),
                                                  frc2::cmd::RunEnd([&drive]
                                                                    { drive.Stop(); }, [&drive]
                                                                    { drive.Stop(); }, {&drive})),
                                              returnHome(true), frc2::cmd::Wait(offenseconfig::kReturnDelay))
                                              .Repeatedly(),
                                          warmShooter())
                                          .FinallyDo([&perception, commitment](bool interrupted)
                                                     {
                            if (!interrupted && perception.MatchState().ownActive.value_or(false))
                              *commitment = OffensiveCommitment::Score; }),
                                      frc2::cmd::None(), [next]
                                      { return next() == OffensiveDecision::Stage; }),
                                  [next]
                                  { return next() == OffensiveDecision::Ferry; }),
                [next]
                { return next() == OffensiveDecision::Score; }),
            frc2::cmd::Wait(offenseconfig::kDecisionDelay))
            .Repeatedly();
    return std::move(cycle).FinallyDo(
        [&drive, m](bool)
        {
          drive.Stop();
          m.rack.Stop();
          m.intake.Stop();
          m.conveyor.Stop();
          m.bottomIndexer.Stop();
          m.topIndexer.Stop();
          m.hood.Stop();
          m.shooter.Stop();
          Phase("idle");
        });
  }
} // namespace season
