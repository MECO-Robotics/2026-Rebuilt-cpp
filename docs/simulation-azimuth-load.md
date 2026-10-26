# Load-dependent azimuth simulation

Azimuth now includes an estimated finite tire contact patch, rather than only
motor inertia/back-EMF and a fixed friction-voltage loss. The model uses each
wheel's solved normal reaction in 3D. An unloaded wheel has zero tire force,
zero ground twisting torque and no remembered carpet shear. Planar simulation
still assumes equal wheel loads.

## Contact model

| Input | Current value | Status |
|---|---:|---|
| Wheel width | 0.0508 m / 2 in | WCP X2 tread nominal width |
| Fore-aft patch length | 0.020 m | Estimate |
| Shear travel at breakaway | 0.001 m | Estimate |
| Tire/carpet CoF | 2.1 | User-selected estimate |
| Output azimuth inertia | 0.01 kg m² | Existing estimate |
| Steering reduction/current limit | 12.1:1 / 30 A | Existing configuration |

The rectangular patch's RMS moment arm is `r = sqrt((width² + length²)/12)`.
This is a lumped quadrature approximation, not a measured pressure distribution.
Its torsional stiffness is `K = mu * normalForce * r² / shearTravel`.

A small elastic twist provides static holding torque instead of discontinuously
switching a fixed drag torque at zero speed. During a physics substep:

```
twistTrial = (previousTwist + dt * patchYawRate) / (1 + dt * rollingSpeed / patchLength)
rawGroundTorque = -K * twistTrial
```

Patch yaw rate includes chassis rotation and azimuth rotation projected onto the
terrain normal. Rolling speed uses the mean of absolute wheel-surface and
fore-aft chassis contact speed. Rolling renews the patch, relaxing stored shear;
stationary steering must strain and then slide the contact patch.

Longitudinal force, lateral force and ground torsion share one friction budget:

```
sqrt(Fx² + Fy² + (groundTorque/r)²) <= mu * normalForce
```

The raw wrench is scaled together when that bound is reached. A plastic return
mapping updates stored twist from the limited torque, dissipating excess shear.
When normal load increases, carried-over twist is reduced to preserve at most the
prior shear moment. This prevents a support-impulse/stiffness jump from creating
unmodeled azimuth spring energy. Unloading reduces stiffness and lifting clears
stored twist entirely.

Wheel speed and steering speed are solved implicitly together, including motor
back-EMF, configured current limits and drive/steer coupling. Steering accelerates
from the combined steering-motor/coupling torque plus the projected ground load.
The chassis receives the opposite actuator reaction and any terrain-normal moment
constrained by the tilted steering axis. Ground torque is not simply added twice
to both module and chassis. This remains a lumped inertia model, not an articulated
gearbox/rotor model.

## Controller and telemetry boundaries

Phoenix still computes applied motor voltage from physical sensor feedback. Motor
constants remain Kraken X60 FOC; current limits remain 50 A drive stator, 55 A drive
supply and 30 A steering. The existing 0.1 V plant loss approximation remains;
controller feedforward `steerKS=0.2` is not the ground-load model. No physical-robot
gains were tuned from this estimate.

With `FRC_SIM_LATENCY_PROBE=1`, `/Diagnostics/Drive/AzimuthLoad` publishes at most
once per 20 ms. Its 25 numbers are a timestamp followed by six values per module:
normal force N, terrain-normal ground torque Nm, elastic twist rad, combined
azimuth actuator torque Nm, plant drive winding current A, plant steer winding
current A. The actuator value is named `steerMotorTorque` in module truth but also
includes drive coupling. These currents are plant estimates, distinct from
Phoenix's internally calculated current signals in `MotorElectrical`.

The pattern driver now requests 20 ms updates consistently for physical truth,
pose, speeds and module states, recording pose/speed timestamps. Previously those
100 ms default subscriptions could distort comparison with 20 ms physical truth,
especially during commanded yaw. Acceptance limits are unchanged.

## Integration contract

Reusable source: MecoSimulation-cpp `8eff394` (initial model `657c27b`). The new
patch settings and module load records **extend the DrivePhysics ABI**. They must
not be mixed with old physics objects. Archive inspection found the two direct
owners in the pinned package: `DrivePhysics.cpp.o` and `DriveSimulationAdapter.o`.
This checkout replaces both complete objects and supplies their matching local
`base/sim/DrivePhysics.h`; the base robot and SimulationWorld interfaces retain
their existing layouts. No vendordep was published or upgraded.

`MECO_DRIVE_PHYSICS_LAYOUT=2` forces coordinated native rebuilds. Native-test
include order explicitly puts local headers before vendordep headers; the linked
wheel-lift regression accesses new load fields and would fail to compile against
the old record layout. Remove the header, whole-object replacements and build
marker together when adopting a compatible published package.

## Validation and physical limits

The focused numerical regression checks static holding, increased resistance with
load, rolling relief, loss of all tire resistance when lifted, shared force/torque
capacity, current limits, actuator torque balance, tilted airborne chassis reaction,
normal-load transient passivity and zero-input energy decay. It fails against the
old point-contact steering implementation. At 0.3 V in the restrained test fixture,
the loaded module settles at 1.25 degrees and 8.05 A plant current; twice the mass
halves its deflection. With 2 m/s rolling it travels about 44.7 degrees over the
one-second steering interval. These are numerical model observations, not robot
measurements.

The patch length, elastic travel, CoF, inertia and pressure shape remain unmeasured.
There is no resolved spike geometry, pressure-area growth with load, tire trail,
camber model or temperature/wear dependence. Sideways slip consumes the shared
friction budget but is not a separate contact-renewal calibration. The model can
expose slower steering and heading drift; passing its force/energy contracts does
not establish real-robot steering accuracy or acceptable autonomous tracking.

[MapleSim's current steering documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/org/ironmaple/simulation/drivesims/SwerveModuleSimulation.html)
and [release notes](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases)
report steering friction omitted/removed; this finite-patch model is not copied
from MapleSim or validated by its behavior. The [WCP module reference](https://docs.wcproducts.com/welcome/frc-build-system/gearboxes/swerve)
provides nominal wheel dimensions, not a measured contact-patch or azimuth torque curve.
