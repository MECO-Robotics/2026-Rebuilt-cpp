#pragma once

#include <array>

namespace season::simulation
{
struct ModuleLocation
{
    double xMeters;
    double yMeters;
};

struct TuningInputs
{
    double massKg;
    double yawMoiKgMetersSquared;
    double frictionCoefficient;
    double wheelRadiusMeters;
    double driveRatio;
    double driveCurrentLimitAmps;
    double motorTorqueConstantNmPerAmp;
    std::array<ModuleLocation, 4> modules;
};

struct TuningResult
{
    double autoTranslationKP;
    double autoRotationKP;
    double maxLinearAcceleration;
    double maxAngularAcceleration;
};

// Initial simulation-only PathPlanner gains from a motor/traction stopping
// envelope. This is an acceleration upper bound, not a steering-load model.
// Steering uses the robot's configured gains with the contact-loaded plant.
TuningResult DeriveSimulationTuning(const TuningInputs& inputs);
} // namespace season::simulation
