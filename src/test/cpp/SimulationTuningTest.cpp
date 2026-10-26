#include "season/SimulationTuning.h"
#include <gtest/gtest.h>
#include <cmath>

TEST(SimulationTuning, ChassisEnvelopeRespectsMotorTractionMassAndYawInertia)
{
    using namespace season::simulation;
    const TuningInputs input{67.6,6.883,2.1,.0508,5.8909090909090915,50,9.37/483,
        {{{.2794,.2794},{.2794,-.2794},{-.2794,.2794},{-.2794,-.2794}}}};
    const auto base=DeriveSimulationTuning(input);
    EXPECT_GT(base.maxLinearAcceleration,0);
    EXPECT_GT(base.maxAngularAcceleration,0);
    auto heavier=input;heavier.massKg*=2;
    const auto heavy=DeriveSimulationTuning(heavier);
    EXPECT_NEAR(heavy.autoTranslationKP,base.autoTranslationKP/std::sqrt(2.),.01);
    EXPECT_DOUBLE_EQ(heavy.autoRotationKP,base.autoRotationKP);
    auto yaw=input;yaw.yawMoiKgMetersSquared*=2;
    EXPECT_NEAR(DeriveSimulationTuning(yaw).autoRotationKP,base.autoRotationKP/std::sqrt(2.),.01);
    auto limited=input;limited.driveCurrentLimitAmps*=.5;
    EXPECT_NEAR(DeriveSimulationTuning(limited).autoTranslationKP,base.autoTranslationKP/std::sqrt(2.),.01);
    auto slippery=input;slippery.frictionCoefficient=.1;
    EXPECT_NEAR(DeriveSimulationTuning(slippery).maxLinearAcceleration,.1*9.80665,1e-9);
}
