# Hard-mounted swerve wheel contact

The robot has hard-mounted modules. The previously used MecoSimulation 2026.0.8
plant incorrectly modeled four vertical spring/damper supports. Its approximately
0.278 m body height was suspension sag in that model, not a property of this robot.
The earlier display-frame correction did not address that modeling error.

## Corrected physical model

The wheel contact points are fixed relative to the chassis by the module locations
and geometric `rideHeight` (now 0.0762 m at bumper centre; see [bumper contact](bumper-contact.md)). Full3D contact queries
find supporting terrain. A unilateral normal-impulse solve shares ground reactions
among contacting wheels, accounting for mass, inertia, center of mass, orientation,
contact lever arms and gravity. Reactions can push but cannot pull a wheel onto the
floor. Airborne/overturned wheels lose their supporting load and traction.

There are no vertical springs, suspension dampers, travel, preload or elastic
energy. A 1 cm speculative ray margin and 0.1 mm penetration stabilization tolerance
are numerical contact settings. They are not suspension travel or load-dependent
compliance. The rigid solver is evaluated on the existing five 4 ms physics steps;
it does not add another plant or change scheduler rate.

This remains an approximate ray-contact tire model: finite tire volume, sidewall
collisions, real tread deformation and dynamic ground-body reaction coupling are
not modeled. A rigid-contact approximation is not a measured tire/robot model.
Mass, center of mass, tire friction and inertia still need physical calibration.

## Integration and pose frames

The original rigid-contact fix, MecoSimulation-cpp `a8bea08`, preserved public
object layouts; legacy suspension fields remain unused. The later
[azimuth load extension](simulation-azimuth-load.md) changes the DrivePhysics
configuration/state layout and requires the matched local header and complete
physics/session adapter objects to be rebuilt together. The published 2026.0.8 package has not been replaced
or republished.

`src/main/cpp/simulation/RigidWheelContactPhysics.cpp` is a desktop-only copy of that
upstream `DrivePhysics.cpp`, replacing its entire static archive object, in the
same way as the existing Phoenix hardware-feedback compatibility bridge. All
methods must remain together to avoid pulling in the old plant. Remove this file
when updating to a published Simulation version containing the rigid-contact fix.

`/Simulation/PhysicsBodyTruth` now settles at its fixed 0.0762 m geometric height,
independent of robot load. `/Simulation/RobotTruth` applies a rigid local -0.0762 m
model-origin offset and therefore reads approximately zero on level carpet. The
offset rotates with the chassis; bump height, pitch, roll and overturning remain
visible. The previous mass/spring-compression offset has been removed.

## Evidence

- Upstream world, drivetrain physics and arena tests passed. The rigid-height
  regression checks 53 kg and 106 kg bodies, fixed geometric height and reaction
  forces supporting their weight. Linking that regression against the original
  spring implementation fails its height check.
- The season build passed desktop and roboRIO Debug/Release targets, with 25 tests
  in each desktop configuration. A native integration regression verifies that
  the linked season drive plant, rather than just the upstream source, does not
  sag under those two loads.
- The real robot executable passed field-aware checks for grounded model/body
  height, blue/red resets, turning, wall contact, field installation and bump
  traversal. It crossed the blue lower bump, reached approximately 0.391 m body
  height, and returned to approximately 0.300 m after leaving it.

Evidence is in `artifacts/rigid-contact-runtime.json` and
`artifacts/rigid-contact-validation.txt`. These are numerical desktop simulation
checks; they do not establish physical tip limits or tire calibration.

Historical 0.300 m body-origin and bump-crossing results above predate the [low bumper shell](bumper-contact.md). The fixed contact model is unchanged; its body origin is now at bumper centre with an offset CoM.
