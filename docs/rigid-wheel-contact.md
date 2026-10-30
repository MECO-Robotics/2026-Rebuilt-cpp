# Wheel contact in simulation

The simulated swerve modules are hard-mounted: they do not have a vertical spring or suspension travel.

- The contact solver determines which wheels are supported and shares the ground load between them.
- A wheel without ground support loses its modeled traction.
- An older version modeled suspension sag that this robot does not have; old height measurements from that version should not be treated as current robot geometry.
- Numerical contact settings and simulated loads are not physical measurements.

See [robot simulation configuration](../src/main/cpp/RobotContainer.cpp) and the [detailed wheel-contact investigation](reference/rigid-wheel-contact.md).
