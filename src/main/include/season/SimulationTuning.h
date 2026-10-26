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
    double steerRatio;
    double steerInertiaKgMetersSquared;
    double driveCurrentLimitAmps;
    double steerCurrentLimitAmps;
    double motorResistanceOhms;
    double motorTorqueConstantNmPerAmp;
    double motorVelocityConstantRadPerSecondPerVolt;
    double nominalVoltage;
    std::array<ModuleLocation, 4> modules;
};

struct TuningResult
{
    double steerKP;
    double steerKD;
    double autoTranslationKP;
    double autoRotationKP;
    double maxLinearAcceleration;
    double maxAngularAcceleration;
};

// Derives model-based initial gains for the simulated drivetrain. Steering
// gains use the Jolt azimuth motor plant; PathPlanner gains use a stopping
// envelope based on motor and traction limits. Hardware gains are unaffected.
TuningResult DeriveSimulationTuning(const TuningInputs& inputs);
} // namespace season::simulation
