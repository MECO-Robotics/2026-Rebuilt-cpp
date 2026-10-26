# REBUILT collision scene and default spawn

Desktop simulation creates a season-specific empty `Arena` on the drivetrain's
existing Jolt world, then installs 40 static field colliders. It does not select
the library's generic practice field. The adapter remains the single physics
owner; its existing ground surface supplies the carpet.

## Included geometry

- Four perimeter walls, with their inner faces at the carpet boundary.
- Two solid hub chassis-contact envelopes.
- Four bumps, each modeled with two thin, 15-degree sloped plates, rather than
  an impassable wall across the bump footprint.
- Four trench inner supports and overhead arms, retaining the low passage.
- Simplified tower bases, uprights and rungs, plus depot perimeter barriers.

Field length/width and hub locations use the pinned
[WPILib 2026.2.1 welded-field layout](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json).
Bump and trench dimensions come from the
[FIRST game manual, sections 5.4-5.8](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm).
The [MapleSim v0.4.0-beta REBUILT map](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/v0.4.0-beta/project/src/main/java/org/ironmaple/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.java)
provides a reference for the layout and simplified tower placement; mirrored
trench geometry avoids that map's duplicated fourth support. Unlike its default
2D ramp-blocking rectangles, these bumps are sloped 3D contacts.

This is simplified collision geometry, not official CAD meshing: tower placement,
depot frames, wall heights and chassis-contact envelopes are approximations.
Hub openings/netting, gate/outpost details, moving arms, decorative geometry and
mechanism collision bodies are omitted. Fuel still uses the separate season
model, not Jolt rigid-body pieces. Trench clearance represents the current
chassis box, not the full robot/superstructure. Collision acceptance does not
establish physical tip limits, inspection legality or full match rules.

## Starting pose and reset

| Alliance | X | Y | Heading |
| --- | ---: | ---: | ---: |
| Blue, or unknown on startup | 3.650 m | 0.639 m | 0 degrees |
| Red | 12.891 m | 7.430 m | 180 degrees |

The modeled .86 m square chassis stays within the field, overlaps the alliance
starting line and clears ground obstacles. It is centered in a trench lane, with clearance for its 45-degree swept footprint; its current
chassis collider clears the overhead arm. The start-space check includes the
modeled bumper footprint and a 2 cm obstacle margin, and rejects `(0,0)` and hub
interiors. It is run before resetting physics.

Startup, autonomous scene initialization, and **Back/View in Teleop** use the
same alliance-aware spawn. An alliance selected after startup takes effect on
the next scene reset. Start/Menu remains only the driver heading reset.

The deferred odometry reconciliation now measures motion relative to the actual
physical reset pose, rather than `(0,0)`. This prevents double-adding the new
spawn translation and preserves the existing handling of subsequent autonomous
odometry seeds. An autonomous routine can still deliberately seed a different
odometry frame; this does not teleport physical truth to that path start. Such
routines must be aligned with physical field coordinates before collision-aware
trajectory acceptance. This change validates driving/reset, not all season autos.

`/SmartDashboard/FieldSimulation/CollidersEnabled` and `ColliderCount` report the
installed scene.

### Ground-level model origin

`/Simulation/RobotTruth` is the visualization/model pose, with its local origin
at floor level when grounded. `/Simulation/PhysicsBodyTruth` retains the raw
body-origin pose. Its flat-ground Z is the fixed `rideHeight` geometry, currently
0.0762 m at the bumper centre (see [bumper contact](bumper-contact.md)). Hard-mounted modules have no modeled vertical suspension, and changing
mass does not change that geometric height.

A fixed local transform maps the body frame to the model's floor reference.
It rotates with the chassis, so pitch/roll, bump traversal, jumping and overturning
remain visible. It does not clamp Z or change collision bodies. The same geometric
offset applies in planar mode. Numerical contact tolerances are not suspension
travel. See [the rigid wheel-contact model](rigid-wheel-contact.md).

The drive, pattern and field runtime checks subscribe to the raw body topic
for physical contact/traction diagnostics. Earlier artifacts using 0.278 m body
height came from the now-replaced spring model. That model was not representative
of the robot's hard-mounted swerve modules.

## Validation and free-space mode

The final Gradle build passed desktop and roboRIO Debug/Release targets, with
25 tests in each desktop configuration. The field regressions cover rigid support height under different loads, clear
blue/red starts (and rejected invalid starts) and actual powered-body contact
with a hub, using the Jolt solver.

The running robot executable additionally passed the field-runtime checks:
clear initial spawn, blue/red physics-and-odometry resets, real south-wall
contact blocking lateral motion, actual turning at the default spawn, a zero grounded
model height, an installed scene, fixed hard-mount body height, and traversal of
a bump followed by a return to that height. Evidence:
`artifacts/rigid-contact-runtime.json` (earlier runs are retained in
`artifacts/ground-origin-runtime.json` and `artifacts/field-collision-runtime.json`). Repeat after a build with:

```sh
./scripts/sim_headless.sh
# In another terminal, using the documented isolated Python dependencies:
python scripts/field_collision_smoke.py --output artifacts/field-collision-repeat.json
```

The runtime check disables the local simulator and restores the blue scene in
cleanup. Set `FRC_SIM_GUI=1` to include the GUI; its physical joystick producer can
compete with injected test inputs, so use headless mode for the integration test.

For empty-world drive/Choreo regression experiments, explicitly launch with
`FRC_SIM_FIELD_COLLIDERS=0`. This disables only the season field obstacles,
preserves the ground, and restores `(0,0)` as the free-space reset pose. The flag
is read at construction and requires restarting; the dashboard fields are status,
not runtime enable controls. All geometry and spawn changes are excluded from
roboRIO builds.

The earlier bump-crossing evidence below predates the low bumper shell. With the requested 0.25-inch gap, the updated runtime check verifies bumper contact at the bump approach instead; see [bumper contact](bumper-contact.md).
