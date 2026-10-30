# Steering load in simulation

**Azimuth** is a swerve module's steering angle. Turning a wheel on carpet needs more effort when the wheel is carrying weight; the simulator approximates that effect.

- The 3D model uses the normal load on each wheel. An unloaded wheel has no modeled tire contact force.
- Wheel contact patch size, stiffness, friction, and inertia are estimates, not measured carpet behavior.
- Steering response in simulation is useful for comparison, but does not replace hardware tuning.

See [drive configuration](../src/main/cpp/RobotContainer.cpp), [wheel contact](rigid-wheel-contact.md), and the [detailed steering-load record](reference/simulation-azimuth-load.md).
