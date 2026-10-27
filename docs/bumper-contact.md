> Current Full3D configuration uses [compressible noodles with rigid plywood and a solid pan](compliant-bumper-contact.md). The rigid shell and numerical results below describe the historical 2026.0.10 release.

# Bumper contact with terrain and obstacles

The robot now has a hollow, four-sided bumper collision shell, rigidly attached
to the same Jolt body as the drivetrain. There are no independent bumper bodies,
extra robot mass or suspension. Ground, sloped bump plates, low obstacles, walls
and field structures use Jolt collision response, including forces and moments on
the chassis. Wheel support rays remain separate and exclude the robot itself.

## Geometry and material assumptions

Desktop tire/carpet CoF is **2.1**, selected by the user as a simulation estimate
for WCP spiked tread. This is distinct from the bumper material coefficient below.
It is informed by [Spectrum's related SLS-wheel test](https://www.chiefdelphi.com/t/spectrum-3847-build-blog-2024/447471/217),
not a published WCP-1728 measurement. Real-robot PathPlanner configuration is
unchanged. Earlier bumper artifacts used tire CoF 2.9; current validation uses 2.1.

The outer footprint remains 0.86 m square. Per the requested setting, the floor
gap is **0.25 inch (0.00635 m)**. The estimated bumper height is **5.5 inches
(0.1397 m)**, putting the top at 5.75 inches (0.14605 m). FIRST's
[2026 R405 height-zone requirement](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm)
requires supported padding throughout the 2.5–5.75 inch zone; it does not impose
2.5 inches as minimum bumper-to-floor clearance. These envelope choices cover that
zone, not every construction/inspection requirement.

Four compound box segments use an estimated 3.5-inch radial depth. Corners meet
without overlap, and the centre stays open: an obstacle under the robot does not
hit an imaginary solid belly plate. Rounded collision edges use an estimated nominal 1.5-inch (38.1 mm) radius,
bounded to 95% of each segment half dimension. The outer envelope and floor gap
remain unchanged. This rounded profile prevents the former 5 mm corners catching
the bump leading lip. Bumper material friction is 0.65, following
MapleSim's reference bumper coefficient; Jolt combines it with the opposing
material. Restitution is zero. This is **rigid contact**, not a calibrated foam
compression, rebound or damage model. The rounded geometry approximates a padded edge; it does not model foam deflection.

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

At the requested low clearance, the bumper strikes the REBUILT bump before
its wheels. MecoSimulation 2026.0.10 lowers only the HDPE ramp material friction
from 1.0 to 0.010. Jolt combines this with the bumper's 0.65 to give an estimated
pair coefficient of 0.081, previously 0.806. The official ramp geometry is
unchanged. These are model estimates, not measured friction coefficients.

Live full-stick approaches now ride up and lift all four wheels. Complete crossing
is inconsistent: the 0.25-inch shell can high-center between the ramp and carpet,
leaving its tires unloaded. The acceptance check requires actual ramp progress,
body lift and wheel unloading; it separately reports whether the entire bump was
cleared. There are no airborne tire forces, special bump exclusions or lift forces.
The separate tire traction model remains CoF 2.1; HDPE tire traction is not calibrated.

## Implementation and validation

The reusable bumper-ring implementation originated in MecoSimulation-cpp `d3f125e`;
2026.0.10 adds rounded, low-friction bump contact in upstream `84daa28`. A new
`BodySpec::Shape::BumperRing` uses the existing `radius` slot for wall thickness;
public structure/class layouts remain unchanged. Invalid wall thickness and
unknown shapes are rejected. MecoSimulation 2026.0.10 packages the world, drive
physics and matching adapter/header layouts together. Jolt headers and ABI build
settings are internal to that package; Jolt is not linked into roboRIO code.

The focused upstream regression verifies a hollow centre, actual edge ray hits,
a tilted drop onto the floor without deep penetration, passage of sub-clearance
objects, low-obstacle blocking and sloped-plate contact. It fails against the
original world implementation (`artifacts/bumper-original-regression.txt`). Existing
world, drive-physics and arena tests also pass. The actual desktop robot field
run is recorded in `artifacts/bumper-field-runtime.json` and checks floor support,
legal blue/red spawns, rotation, wall contact and low-bumper bump contact. That historical run expected a leading-edge stop;
[current bump validation](../artifacts/bump-contact-validation-2026.0.10.json)
requires sliding up and lifting the chassis. The focused native regression fails
against the published 2026.0.9 archive and passes with the corrected contact model.

These are numerical simulation checks. Actual bumper profile, compression,
friction, ground clearance and robot mass distribution still need measurement.

The full-stick ground stress mode exercises the actual joystick/controller path:

```sh
FRC_SIM_FIELD_COLLIDERS=0 FRC_SIM_LATENCY_PROBE=1 ./scripts/sim_headless.sh
python scripts/drive_pattern_smoke.py --bumper-stress --output artifacts/bumper-ground-stress.json
```

It verifies a >4 m/s approach, a real lean caught below 15 degrees and at most
5 mm numerical penetration of the actual rounded surface. Nominal bounding-box
corners are also reported but do not represent material at rounded edges. The historical 5 mm-radius run approached at 4.51 m/s,
peaked at 4.74 degrees and had 3.72 mm nominal corner penetration (the collider's
rounded corners matter at that scale). This passes the bumper-ground check,
**not** square tracking acceptance. The former elevated-box run reached about
70 degrees. A preceding low-bumper diagnostic reached 6.42 degrees; asynchronous
whole-robot runs need not reproduce exactly the same peak.

Normal moderate square/circle results are in `artifacts/bumper-drive-patterns.json`.
Both circle checks pass; the square's existing yaw-drift check still fails
(0.263 rad versus a 0.15 rad limit). Other square metrics pass. No tracking or yaw
threshold was relaxed to claim that bumper contact fixes steering.

The post-integration [drive dynamics run](../artifacts/bumper-drive-dynamics.json)
also passes release, reversal and disabled-braking regression bands. The full
validation scope and remaining limitations are recorded in
[bumper-validation.txt](../artifacts/bumper-validation.txt).

Current tire-CoF 2.1 validation is summarized in
[tire-cof-2p1-validation.json](../artifacts/tire-cof-2p1-validation.json): all three
dynamics modes, bumper-ground stress and all nine field checks pass. The ground
stress peak is 5.26 degrees; release stopping distance is 3.34 m. This check does
not claim to resolve the separate square yaw-drift limitation.

The 2026.0.10 square/circle repeat is recorded in
[bump-contact-validation-2026.0.10.json](../artifacts/bump-contact-validation-2026.0.10.json).
Both circle checks pass. The square again fails its unchanged yaw-drift limit;
this run reaches 1.10 rad. Ground stress remains a separate passing contact check
(4.33 degrees peak, 1.31 mm actual-surface penetration).
