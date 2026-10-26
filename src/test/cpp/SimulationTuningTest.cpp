#include "season/SimulationTuning.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
season::simulation::TuningInputs RebuiltInputs()
{
    return {53.0,
            6.883,
            2.9,
            .0508,
            5.8909090909090915,
            12.1,
            .01,
            55.0,
            30.0,
            12.0 / 366.0,
            7.09 / 366.0,
            55.0,
            12.0,
            {{{.2794, .2794}, {.2794, -.2794}, {-.2794, .2794}, {-.2794, -.2794}}}};
}
} // namespace

TEST(SimulationTuning, DerivesSteeringAndChassisGainsFromTheirOwnPlantParameters)
{
    using namespace season::simulation;
    const auto input = RebuiltInputs();
    const auto base = DeriveSimulationTuning(input);

    EXPECT_NEAR(base.steerKP, 30.61, .1);
    EXPECT_DOUBLE_EQ(base.steerKD, 0.0);
    EXPECT_NEAR(base.autoTranslationKP, 15.2, .2);
    EXPECT_NEAR(base.autoRotationKP, 10.6, .2);
    EXPECT_GT(base.maxLinearAcceleration, 0);
    EXPECT_GT(base.maxAngularAcceleration, 0);

    auto heavier = input;
    heavier.massKg *= 2;
    const auto heavyTune = DeriveSimulationTuning(heavier);
    EXPECT_NEAR(heavyTune.autoTranslationKP, base.autoTranslationKP / std::sqrt(2.0), .01);
    EXPECT_DOUBLE_EQ(heavyTune.steerKP, base.steerKP);
    EXPECT_DOUBLE_EQ(heavyTune.steerKD, base.steerKD);

    auto higherSteerInertia = input;
    higherSteerInertia.steerInertiaKgMetersSquared *= 2;
    const auto inertiaTune = DeriveSimulationTuning(higherSteerInertia);
    EXPECT_LT(inertiaTune.steerKP, base.steerKP);
    EXPECT_DOUBLE_EQ(inertiaTune.steerKD, 0.0);

    auto largerYawMoi = input;
    largerYawMoi.yawMoiKgMetersSquared *= 2;
    const auto yawTune = DeriveSimulationTuning(largerYawMoi);
    EXPECT_NEAR(yawTune.autoRotationKP, base.autoRotationKP / std::sqrt(2.0), .01);
}
