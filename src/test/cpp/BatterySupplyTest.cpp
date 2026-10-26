#ifndef __FRC_ROBORIO__
#include "meco/sim/BatterySupply.h"
#include "base/SimulationRuntime.h"
#include <frc/RobotController.h>
#include <hal/HAL.h>
#include <gtest/gtest.h>

TEST(BatterySupply, SharedPlantLoadsSagAndRecoverOnRuntimeAndRio) {
    HAL_Initialize(500,0);
    base::simulation::SimulationRuntime runtime;
    double chassisCurrent=100, mechanismCurrent=50, observedVoltage=0;
    auto chassis=runtime.Register({[&](double){observedVoltage=runtime.BusVoltage();},
                                   {},{},{},[&]{return chassisCurrent;}});
    auto mechanism=runtime.Register({{},{},{},{},[&]{return mechanismCurrent;}});
    runtime.StepRobotPeriod();
    EXPECT_DOUBLE_EQ(meco::sim::UpdateBatterySupply(runtime),10.5);
    for(int i=0;i<25;++i){runtime.StepRobotPeriod();meco::sim::UpdateBatterySupply(runtime);}
    EXPECT_NEAR(frc::RobotController::GetBatteryVoltage().value(),9.0,1e-6);
    runtime.StepRobotPeriod();
    EXPECT_NEAR(observedVoltage,9.0,1e-6);
    chassisCurrent=mechanismCurrent=0;
    runtime.StepRobotPeriod();
    for(int i=0;i<25;++i){runtime.StepRobotPeriod();meco::sim::UpdateBatterySupply(runtime);}
    EXPECT_NEAR(frc::RobotController::GetBatteryVoltage().value(),12.0,1e-6);
    chassisCurrent=1000;
    runtime.StepRobotPeriod();
    for(int i=0;i<25;++i){runtime.StepRobotPeriod();meco::sim::UpdateBatterySupply(runtime);}
    EXPECT_GE(runtime.BusVoltage(),0.0);
    EXPECT_LT(runtime.BusVoltage(),1e-6);
    EXPECT_THROW(meco::sim::UpdateBatterySupply(runtime,{12,-.02,0}),std::invalid_argument);
    frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12});
    frc::sim::RoboRioSim::SetVInCurrent(units::ampere_t{0});
}

TEST(BatterySupply, HighConductanceLoadConvergesInsteadOfAlternatingSupply) {
    HAL_Initialize(500,0);
    base::simulation::SimulationRuntime runtime;
    // A 10 mOhm load on a 20 mOhm battery has a 4 V circuit equilibrium.
    // Direct delayed feedback alternates between 0 and 12 V indefinitely.
    auto load=runtime.Register({{},{},{},{},[&]{return runtime.BusVoltage()/.010;}});
    for(int i=0;i<50;++i){runtime.StepRobotPeriod();meco::sim::UpdateBatterySupply(runtime);}
    EXPECT_NEAR(runtime.BusVoltage(),4.0,1e-6);
    EXPECT_NEAR(frc::RobotController::GetBatteryVoltage().value(),4.0,1e-6);
    frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{12});
    frc::sim::RoboRioSim::SetVInCurrent(units::ampere_t{0});
}
#endif
