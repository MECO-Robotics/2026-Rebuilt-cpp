#include "season/SimulationTuning.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace season::simulation
{
namespace
{
constexpr double kGravityMetersPerSecondSquared = 9.80665;
constexpr double kTranslationErrorScaleMeters = .08;
constexpr double kHeadingErrorScaleRadians = .50;

void RequirePositive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string{"Simulation tuning requires positive finite "} + name);
}
} // namespace

TuningResult DeriveSimulationTuning(const TuningInputs& in)
{
    for (const auto& [value, name] : std::array<std::pair<double, const char*>, 7>{
             {{in.massKg, "mass"}, {in.yawMoiKgMetersSquared, "yaw MOI"},
              {in.frictionCoefficient, "friction coefficient"}, {in.wheelRadiusMeters, "wheel radius"},
              {in.driveRatio, "drive ratio"}, {in.driveCurrentLimitAmps, "drive current limit"},
              {in.motorTorqueConstantNmPerAmp, "motor torque constant"}}})
        RequirePositive(value, name);
    for (const auto& module : in.modules)
        if (!std::isfinite(module.xMeters) || !std::isfinite(module.yMeters))
            throw std::invalid_argument("Simulation tuning requires finite module locations");

    // Current-limited motor force is further capped by per-wheel traction.
    const double driveTorque = in.driveRatio * in.motorTorqueConstantNmPerAmp * in.driveCurrentLimitAmps;
    const double motorForce = driveTorque / in.wheelRadiusMeters;
    const double tractionForce = in.frictionCoefficient * in.massKg * kGravityMetersPerSecondSquared / 4.0;
    const double moduleForce = std::min(motorForce, tractionForce);
    const double maxLinearAcceleration = 4.0 * moduleForce / in.massKg;
    double maxYawTorque = 0;
    for (const auto& module : in.modules)
        maxYawTorque += std::hypot(module.xMeters, module.yMeters) * moduleForce;
    const double maxAngularAcceleration = maxYawTorque / in.yawMoiKgMetersSquared;
    if (!std::isfinite(maxLinearAcceleration) || maxLinearAcceleration <= 0 ||
        !std::isfinite(maxAngularAcceleration) || maxAngularAcceleration <= 0)
        throw std::invalid_argument("Simulation tuning requires nonzero drive and yaw authority");

    // For proportional pose feedback, v=Kp*error. At the chosen error scale,
    // v^2/(2*a) equals that error, so the command remains inside the available
    // acceleration stopping envelope.
    return {std::sqrt(2.0 * maxLinearAcceleration / kTranslationErrorScaleMeters),
            std::sqrt(2.0 * maxAngularAcceleration / kHeadingErrorScaleRadians),
            maxLinearAcceleration,
            maxAngularAcceleration};
}
} // namespace season::simulation
