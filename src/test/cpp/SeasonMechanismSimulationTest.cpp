#include "base/Mechanism.h"
#include "season/MechanismConstants.h"
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/DriverStation.h>
#include <hal/HAL.h>

namespace {
void EnableSim(bool enabled) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetEnabled(enabled);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();
}
}
TEST(SeasonSimulation, HoodHighReductionTracksSmallPositionWithoutNumericalInstability) {
  EnableSim(true);
  auto io=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,{{33},{false},105,40,""},.01);
  auto* plant=io.get();
  base::PositionJoint hood("HoodStability",std::move(io),season::hardware::SimulationGainsFor(season::hardware::kHoodGains),1,base::Gravity::Cosine);
  hood.SetPosition(.020);
  double maxPosition=0;
  for(int step=0;step<1000;++step) {
    hood.Periodic();
    plant->AdvanceSimulation(units::second_t{.02});
    ASSERT_TRUE(std::isfinite(hood.Position()));
    ASSERT_TRUE(std::isfinite(hood.Velocity()));
    maxPosition=std::max(maxPosition,std::abs(hood.Position()));
  }
  EXPECT_LT(maxPosition,.05);
  EXPECT_NEAR(hood.Position(),.020,.003);
  EnableSim(false);
}
