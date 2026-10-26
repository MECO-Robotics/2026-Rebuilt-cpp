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
constexpr double kPi = 3.14159265358979323846;
constexpr double kTranslationErrorScaleMeters = .08;
constexpr double kHeadingErrorScaleRadians = .50;
constexpr double kSteerLinearizationErrorRadians = 10.0 * kPi / 180.0;
// Phoenix consumes sensor feedback asynchronously, once per robot-period batch
// of physics substeps. Keep the dominant response slower than that feedback delay.
constexpr double kSteerSettlingSeconds = .15;

void RequirePositive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string{"Simulation tuning requires positive finite "} + name);
}
} // namespace

TuningResult DeriveSimulationTuning(const TuningInputs& in)
{
    for (const auto& [value, name] : std::array<std::pair<double, const char*>, 12>{
             {{in.massKg, "mass"},
              {in.yawMoiKgMetersSquared, "yaw MOI"},
              {in.frictionCoefficient, "friction coefficient"},
              {in.wheelRadiusMeters, "wheel radius"},
              {in.driveRatio, "drive ratio"},
              {in.steerRatio, "steer ratio"},
              {in.steerInertiaKgMetersSquared, "steer inertia"},
              {in.driveCurrentLimitAmps, "drive current limit"},
              {in.steerCurrentLimitAmps, "steer current limit"},
              {in.motorResistanceOhms, "motor resistance"},
              {in.motorTorqueConstantNmPerAmp, "motor torque constant"},
              {in.motorVelocityConstantRadPerSecondPerVolt, "motor velocity constant"}}})
        RequirePositive(value, name);
    RequirePositive(in.nominalVoltage, "nominal voltage");
    for (const auto& module : in.modules)
        if (!std::isfinite(module.xMeters) || !std::isfinite(module.yMeters))
            throw std::invalid_argument("Simulation tuning requires finite module locations");

    // The linearized Jolt azimuth plant is J*theta'' + B*theta' = A*V,
    // where theta is module output radians. Phoenix gains use module rotations.
    // Place the nominal plant pole with proportional feedback, including the
    // electrical damping B. Add a small feedback lead below for the asynchronous
    // Phoenix/robot-period transport boundary.
    const double steerTorquePerVolt = in.steerRatio * in.motorTorqueConstantNmPerAmp / in.motorResistanceOhms;
    const double steerElectricalDamping = in.steerRatio * in.steerRatio * in.motorTorqueConstantNmPerAmp /
                                          (in.motorResistanceOhms * in.motorVelocityConstantRadPerSecondPerVolt);
    const double desiredSlowPole = 4.0 / kSteerSettlingSeconds;
    const double maxLinearKp = 2.0 * kPi * in.steerCurrentLimitAmps * in.motorResistanceOhms /
                               kSteerLinearizationErrorRadians;
    const double maxKpRad = maxLinearKp / (2.0 * kPi);
    const double discriminant = std::max(0.0, steerElectricalDamping * steerElectricalDamping -
                                                  4.0 * in.steerInertiaKgMetersSquared * steerTorquePerVolt * maxKpRad);
    const double currentLimitedSlowPole =
        (steerElectricalDamping - std::sqrt(discriminant)) / (2.0 * in.steerInertiaKgMetersSquared);
    const double slowPole = std::min({desiredSlowPole,
                                      steerElectricalDamping / (2.0 * in.steerInertiaKgMetersSquared),
                                      currentLimitedSlowPole});
    const double steerKpRad = in.steerInertiaKgMetersSquared * slowPole *
                              (steerElectricalDamping / in.steerInertiaKgMetersSquared - slowPole) /
                              steerTorquePerVolt;
    const double steerKP = 2.0 * kPi * steerKpRad;
    // Lead the asynchronous feedback by one robot period. This damps large
    // azimuth steps without sacrificing stiffness against drive/steer coupling.
    const double steerKD = steerKP * .02;
    if (!std::isfinite(steerKP) || steerKP <= 0 || steerKP > maxLinearKp ||
        !std::isfinite(currentLimitedSlowPole) || currentLimitedSlowPole <= 0)
        throw std::invalid_argument("Derived steering gains exceed the simulated actuator limits");

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
    return {steerKP,
            steerKD,
            std::sqrt(2.0 * maxLinearAcceleration / kTranslationErrorScaleMeters),
            std::sqrt(2.0 * maxAngularAcceleration / kHeadingErrorScaleRadians),
            maxLinearAcceleration,
            maxAngularAcceleration};
}
} // namespace season::simulation
