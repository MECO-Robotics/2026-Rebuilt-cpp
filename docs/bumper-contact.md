# Bumper contact with terrain and obstacles

The robot now has a hollow, four-sided bumper collision shell, rigidly attached
to the same Jolt body as the drivetrain. There are no independent bumper bodies,
extra robot mass or suspension. Ground, sloped bump plates, low obstacles, walls
and field structures use Jolt collision response, including forces and moments on
the chassis. Wheel support rays remain separate and exclude the robot itself.

## Geometry and material assumptions

The outer footprint remains 0.86 m square. Per the requested setting, the floor
gap is **0.25 inch (0.00635 m)**. The estimated bumper height is **5.5 inches
(0.1397 m)**, putting the top at 5.75 inches (0.14605 m). FIRST's
[2026 R405 height-zone requirement](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm)
requires supported padding throughout the 2.5–5.75 inch zone; it does not impose
2.5 inches as minimum bumper-to-floor clearance. These envelope choices cover that
zone, not every construction/inspection requirement.

Four compound box segments use an estimated 3.5-inch radial depth. Corners meet
without overlap, and the centre stays open: an obstacle under the robot does not
hit an imaginary solid belly plate. Rounded collision edges use a 5 mm convex
radius, below the 6.35 mm floor gap. Bumper material friction is 0.65, following
MapleSim's reference bumper coefficient; Jolt combines it with the opposing
material. Restitution is zero. This is **rigid contact**, not a calibrated foam
compression, rebound or damage model. A real foam bumper may compress and climb
an edge that this rigid approximation catches.

The former solid box was centred 0.30 m above the floor with 0.12 m half-height:
its bottom was 0.18 m high. That missed low bumper strikes and let the robot lean
much farther before body contact with the ground.

## Coordinate frames

The body origin is now the bumper centre, **0.0762 m above flat ground**. Its
relative CoM offset is +0.2238 m, preserving the previous **assumed 0.30 m world
CoM height** and the same 67.6 kg mass and inertia. This change lowers collision
geometry, not the physical CoM. Fixed wheel contact rays end at the floor.
`/Simulation/RobotTruth` still uses a floor-level origin; `/Simulation/PhysicsBodyTruth`
reports the bumper-centre body origin. The dashboard publishes
`FieldSimulation/GroundedBodyHeight`, `BumperBottomHeight` and `BumperTopHeight`.
Runtime support checks read the configured origin height instead of assuming 0.30 m.

At the requested low clearance, the bumper strikes the REBUILT bump's leading
plate before the wheels reach it. The field integration check verifies that the
shell stops near x=3.63 m on the lower blue bump approach, rather than requiring
the old elevated-box model's unobstructed bump crossing. The low gap has a real
modeled traversal cost; no special collision exclusion is used for bumps.

## Implementation and validation

The reusable implementation is MecoSimulation-cpp `d3f125e`. A new
`BodySpec::Shape::BumperRing` uses the existing `radius` slot for wall thickness;
public structure/class layouts remain unchanged. Invalid wall thickness and
unknown shapes are rejected. The season compatibility copies replace complete
static archive objects and carry upstream GPL-3.0 attribution. The header extension
preserves the pinned vendordep ABI; no package was published or upgraded.

The world replacement needs Jolt headers matching the pinned binary. Gradle's
`prepareJoltHeaders` fetches revision `0373ec0dd762e4bc2f6acdb08371ee84fa23c6db`,
checks its SHA-256 and extracts headers into `build/jolt-headers`. A first desktop
build therefore needs network access. Jolt Debug/Release layout macros match the
respective vendordep libraries; Jolt is not linked into roboRIO code.

The focused upstream regression verifies a hollow centre, actual edge ray hits,
a tilted drop onto the floor without deep penetration, passage of sub-clearance
objects, low-obstacle blocking and sloped-plate contact. It fails against the
original world implementation (`artifacts/bumper-original-regression.txt`). Existing
world, drive-physics and arena tests also pass. The actual desktop robot field
run is recorded in `artifacts/bumper-field-runtime.json` and checks floor support,
legal blue/red spawns, rotation, wall contact and low-bumper bump contact.

These are numerical simulation checks. Actual bumper profile, compression,
friction, ground clearance and robot mass distribution still need measurement.

The full-stick ground stress mode exercises the actual joystick/controller path:

```sh
FRC_SIM_FIELD_COLLIDERS=0 FRC_SIM_LATENCY_PROBE=1 ./scripts/sim_headless.sh
python scripts/drive_pattern_smoke.py --bumper-stress --output artifacts/bumper-ground-stress.json
```

It verifies a >4 m/s approach, a real lean caught below 15 degrees and at most
5 mm numerical corner penetration. The recorded run approached at 4.51 m/s,
peaked at 4.74 degrees and had 3.72 mm nominal corner penetration (the collider's
rounded corners matter at that scale). This passes the bumper-ground check,
**not** square tracking acceptance. The former elevated-box run reached about
70 degrees. A preceding low-bumper diagnostic reached 6.42 degrees; asynchronous
whole-robot runs need not reproduce exactly the same peak.

Normal moderate square/circle results are in `artifacts/bumper-drive-patterns.json`.
Both circle checks pass; the square's existing yaw-drift check still fails
(0.263 rad versus a 0.15 rad limit). Other square metrics pass. No tracking or yaw
threshold was relaxed to claim that bumper contact fixes steering.
