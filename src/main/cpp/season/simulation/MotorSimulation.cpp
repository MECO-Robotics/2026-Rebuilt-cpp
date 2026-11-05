#include "season/simulation/MotorSimulation.h"
#ifndef __FRC_ROBORIO__
#include "season/simulation/MechanismSimulationConfig.h"
#include "season/simulation/SimulationConfig.h"
#include "meco/sim/MotorSimulationFactories.h"
#include <vector>
#endif

base::MotorIOSelector::SimulationProvider season::simulation::MotorProvider(std::shared_ptr<FuelMotorLoads> loads)
{
#ifndef __FRC_ROBORIO__
  using namespace season::hardware;
  using namespace season::simulation::motorconfig;
  auto loaded = [](bool neo) {
    auto model = meco::sim::LoadedMotorModelFromStallAndFreeSpeed(
        kNominalVoltage, neo ? kNeoStallCurrentAmps : kKrakenStallCurrentAmps,
        neo ? kNeoStallTorqueNm : kKrakenStallTorqueNm,
        neo ? kNeoFreeSpeedRpm : kKrakenFreeSpeedRpm);
    model.viscousDragNmPerRadPerSecond = kViscousDrag;
    model.maximumPhysicsStepSeconds = kMaxPhysicsStepSeconds;
    model.maximumUpdateSeconds = kMaxUpdateSeconds;
    model.loadDirectionSpeedThresholdRadPerSecond = kLoadDirectionSpeedThreshold;
    return model;
  };
  const auto neo = loaded(true);
  const auto kraken = loaded(false);
  using Entry = meco::sim::MotorSimulationEntry;
  const auto entry = [loads](base::Vendor vendor, const FlywheelConfig &config,
                             const meco::sim::LoadedMotorModel &model, std::size_t index) {
    return Entry{vendor, config.motor.canBus, config.motor.ids.front(), MotorInertia(config.role),
                 model, [loads, index] { return loads->torque[index]; }};
  };
  return meco::sim::MakeMotorSimulationProvider({
      {base::Vendor::SparkMax, "", kRackCanId, kRackInertia, std::nullopt, {}},
      {base::Vendor::SparkMax, "", kHoodCanId, kHoodInertia, std::nullopt, {}},
      {base::Vendor::SparkMax, kIntake.motor.canBus, kIntake.motor.ids.front(),
       MotorInertia(kIntake.role), std::nullopt, {}},
      entry(base::Vendor::SparkMax, kConveyor, neo, 0),
      entry(base::Vendor::TalonFX, kBottomIndexer, kraken, 1),
      entry(base::Vendor::SparkMax, kTopIndexer, neo, 2),
      entry(base::Vendor::TalonFX, kShooter, kraken, 3)});
#else
  (void)loads;
  return {};
#endif
}
