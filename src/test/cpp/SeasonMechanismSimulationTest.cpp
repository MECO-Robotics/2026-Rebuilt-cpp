#include "base/Mechanism.h"
#include "season/HardwareAdapter.h"
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>

namespace {
void EnableSim(bool enabled) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetEnabled(enabled);
  frc::sim::DriverStationSim::NotifyNewData();
}
}
TEST(SeasonSimulation, HoodHighReductionTracksSmallPositionWithoutNumericalInstability) {
  EnableSim(true);
  base::PositionJoint hood("HoodStability",base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,
      {{33},{false},105,40,""},.01),season::SimulationGainsFor(season::hardware::kHoodGains),1,base::Gravity::Cosine);
  hood.SetPosition(.020);
  double maxPosition=0;
  for(int step=0;step<1000;++step) {
    hood.Periodic();
    ASSERT_TRUE(std::isfinite(hood.Position()));
    ASSERT_TRUE(std::isfinite(hood.Velocity()));
    maxPosition=std::max(maxPosition,std::abs(hood.Position()));
  }
  EXPECT_LT(maxPosition,.05);
  EXPECT_NEAR(hood.Position(),.020,.003);
  EnableSim(false);
}
TEST(SeasonSimulation, RackUsesPhysicalNineToOneReductionAndDrumCircumference) {
  EnableSim(true);
  constexpr double circumference=2*std::numbers::pi*season::hardware::kRackDrumRadius;
  constexpr double outputInertia=15*.45359237*.0127*.0127;
  base::PositionJoint rack("RackStability",base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,
      {{21},{false},season::hardware::kRackGearRatio*circumference,40,""},outputInertia),
      season::SimulationGainsFor(season::hardware::kRackGains),circumference);
  rack.SetPosition(.30);
  double maxPosition=0;
  for(int step=0;step<1000;++step) {
    rack.Periodic();
    ASSERT_TRUE(std::isfinite(rack.Position()));
    ASSERT_TRUE(std::isfinite(rack.Velocity()));
    maxPosition=std::max(maxPosition,std::abs(rack.Position()));
  }
  EXPECT_LT(maxPosition,.36);
  EXPECT_NEAR(rack.Position(),.30,.02);
  EnableSim(false);
}
