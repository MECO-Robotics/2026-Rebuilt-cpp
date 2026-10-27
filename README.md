# 2026 REBUILT — C++

MECO Robotics team 8324's native WPILib 2026 C++ robot project. Shared base code is consumed from the pinned [MecoBase vendordep](https://github.com/MECO-Robotics/MecoBase-cpp).

The original Java projects are preserved in sibling folders. Shared C++ base
classes come from the `MecoBase` vendordep; season code and hardware mappings
remain in this repository. See [the port contract](docs/port-contract.md) for
source commits, units and validation boundaries.

For architecture-aware Codex assistance, use the repository skill [`frc-base-project`](.agents/skills/frc-base-project/SKILL.md), or invoke `$frc-base-project`. Test guidance lives in `AGENTS.md`.

For device configuration, commissioning, and diagnosis, use [`frc-hardware`](.agents/skills/frc-hardware/SKILL.md) or invoke `$frc-hardware`.

## Build and simulate

Install WPILib 2026.2.1 with its C++ toolchains. This project uses the published
[MecoBase 2026.2.1 vendordep](https://meco-robotics.github.io/MecoBase-cpp/MecoBase.json)
and the matching-year [MecoSimulation 2026.0.10 vendordep](https://meco-robotics.github.io/MecoSimulation-cpp/MecoSimulation.json).
The current checkout resolves both from their public Maven URLs; the build,
Debug/Release tests, and localhost desktop drive smoke against MecoSimulation
2026.0.6 are recorded in the [public-package validation record](artifacts/public-multivendordep-validation-2026.0.6.json)
and the [drive smoke output](artifacts/season-drive-smoke-public-2026.0.6.json).
For a clean project, install MecoBase and then MecoSimulation through
**WPILib → Manage Vendor Libraries → Install new libraries (online)**. MecoBase
alone supports roboRIO builds. MecoSimulation adds Linux and Windows desktop
physics libraries. Choreo remains a separate season dependency.

```sh
./gradlew build --max-workers=2
./gradlew simulateNative
```

The build includes desktop and roboRIO debug/release executables and GoogleTest regressions. Deployment is `./gradlew deploy` and writes to team 8324's roboRIO. This port has not been deployed or commissioned on hardware.

Desktop simulation now enables the REBUILT field colliders by default. Startup
and Back/View reset use a clear alliance start, not `(0,0)`. See
[the field scene and validation](docs/simulation-field.md). For the field-aware
runtime check, launch `FRC_SIM_LATENCY_PROBE=1 ./scripts/sim_headless.sh`, then run
`python scripts/field_collision_smoke.py --output artifacts/field-collision-repeat.json`
with the Python dependencies below installed.

For the free-space drive, mechanism and Choreo checks below, start
`FRC_SIM_FIELD_COLLIDERS=0 ./scripts/sim_headless.sh` in one terminal after
building. This deliberately restores the empty-world origin/reset contract;
otherwise field contacts invalidate open-loop speed and trajectory assumptions.
In another terminal:

```sh
python3 -m venv /tmp/frc-cpp-sim-venv
/tmp/frc-cpp-sim-venv/bin/pip install 'pyntcore>=2026,<2027' 'robotpy-wpimath>=2026,<2027' 'websockets>=14,<17'
/tmp/frc-cpp-sim-venv/bin/python scripts/sim_smoke.py --max-speed 5.24 --reset-button 8 --reset-enabled --output artifacts/season-drive-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_mechanism_smoke.py --output artifacts/season-mechanism-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_auto_smoke.py --output artifacts/season-auto-smoke.json
```

Drivetrain acceleration/braking inputs, the FOC/current-limit correction and the measured asymmetric response are documented in [simulation dynamics](docs/simulation-dynamics.md).

These scripts use localhost HAL simulation and leave the simulator disabled. Desktop plants now share a [load-dependent battery supply](docs/simulation-battery.md) using aggregate supply current (12 V, 20 mΩ estimate). Steering now uses the configured robot gains (`kP=100`, `kD=0.5`) with the [contact-loaded azimuth plant](docs/simulation-azimuth-load.md); the old unloaded-motor steering estimate has been removed. Phoenix firmware computes voltages from physical feedback, and its separate built-in plants are not stepped. Simulation-only PathPlanner gains still use a motor/traction acceleration upper bound with mass, yaw inertia, module locations, and stopping-error scales. These numerical estimates do not calibrate the physical robot. The drive smoke checks small-stick response, module azimuth travel, floor support, physical rotation, tracking and disable behavior.

MecoSimulation owns the desktop physics, hardware feedback, diagnostics, battery helper, and 2026 field collider map. This robot consumes its public headers and archive without source or header overrides; see the [ownership boundary](docs/simulation-vendordep-boundary.md). Scene resets reconcile odometry after five robot loops, allowing asynchronous sensor feedback to arrive without blocking the scheduler. Any autonomous pose seed and subsequent motion in that frame are preserved. This one-time correction is limited to scene reset; driving never continuously copies truth into odometry.

The mechanism smoke checks button bindings and motor outputs; the auto smoke checks the Choreo chooser/trajectory endpoint. Simulated rollers can coast after voltage is removed; cancellation checks applied output rather than assuming an immediate mechanical stop. Fuel collection and projectile telemetry appears under `FieldSimulation/`.

For changing joystick directions, with the same headless simulator running:

```sh
/tmp/frc-cpp-sim-venv/bin/python scripts/drive_pattern_smoke.py --output artifacts/drive-patterns.json
```

This runs four cardinal square segments, a circular left-stick sweep, and a
circular sweep with rotation. Each pattern temporarily selects the no-op autonomous and enters it briefly to
invoke the existing scene reset, then runs teleop, disables, and restores the
previous chooser selection in cleanup. Run it only
against the local desktop simulator. It compares physical position differences,
encoder-derived field velocity, odometry pose/heading, module targets, floor
support and disabled motion. It allows 0.75 s for current-limited 90-degree corner
steps and checks continuous tracking thereafter. Per-axis joystick deadbands mean
a circular stick sweep is not an exact geometric circle. The physics/firmware boundary was compared with
[MapleSim's CTRE adapter](https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim):
the external plant supplies encoder/gyro feedback and replaces Phoenix's built-in
plant. MapleSim itself was not run for this validation. Saved numerical results
are in [the drivetrain validation record](artifacts/drivetrain-simulation-validation.json).

## Robot behavior

The season container supplies the Java robot's CAN IDs, bus names, gear ratios, encoder offsets, camera transform and mechanism presets. Linear rack positions are metres; hood positions are rotations; flywheel rates are rotations/second. The rack's Java motor-rotations-per-metre conversion is preserved through a physical 9:1 output ratio and drum circumference.

Pilot controls: left stick drives translation, right X rotates, Start resets driver forward without changing field localization, A aims and continuously applies interpolated hood/flywheel setpoints while retaining translation (release stops hood and shooter), right trigger acquires, left trigger ejects, right bumper agitates and feeds after flywheel readiness. X selects hub/calibration, Y ferry and B idle. Copilot X/Y/B share presets; A selects trench; POV controls the intake. Actions are gated to enabled teleop. Presets latch until replaced; feed stops on cancellation.

Shooting calculations preserve all eight calibration rows, shooter exit/yaw, alliance/tag target selection, ferry trajectory exclusion, readiness ordering and volley latch/debounce. Calibration dashboard values are bounded. Autonomous includes copied PathPlanner assets, named mechanism commands, a stationary vision-initialized shot and native Choreo `LeftBlueBump` followed by a shot. Marker shots do not claim drivetrain ownership; they check the path's current heading rather than overriding the path follower.

Absolute vision comes from Limelight; PhotonVision is available as a reusable base adapter. Stationary autonomous shooting requires a recently accepted field pose; camera heartbeat alone is insufficient. Path-following autos seed their own field pose.

To check hub aiming, run the field-enabled headless simulator and then
`python scripts/autoaim_smoke.py` with the Python dependencies above. This checks
both alliances before and after Start, interpolated hood/flywheel targets and
shooter cleanup on release, plus driver-relative translation command vectors with
and without A held, and disables simulation inputs on exit. The original Start
binding reset absolute odometry heading and caused large physical aiming errors;
Start now stores a separate driver frame. Numerical results are saved in
`artifacts/autoaim-validation.json`; this does not establish hardware alignment.

## Diagnostics

In enabled Test mode, use SmartDashboard `SystemCheck/Armed`, `SystemCheck/SetupConfirmed`, then `SystemCheck/Start`; `SystemCheck/Abort` cancels. The runner reserves all mechanism requirements, checks DS/FMS/e-stop/brownout/voltage interlocks, counts down, evaluates connectivity and bounded mechanism stages, and neutralizes on interruption or mode loss. Reports are JSON on the dashboard and `/home/lvuser/systemcheck-latest.json` (local file in desktop simulation). Read `docs/cpp-port-status.md` before using this replacement for the Java pit-check system.

## Upstream updates

```sh
./gradlew vendordep --update MecoBase
```

Review the vendordep version change, then rebuild. Shared base sources are
published in the vendor artifact and are not copied into this project.

The drive smoke compares field-relative commands with field-relative measured
velocity and reports translational peak speed separately from angular rate.
Its small-stick peak guard uses physical chassis motion; encoder speed also
includes wheel slip and steering/drive coupling during module reorientation.
The telemetry subscription requests 20 ms samples so optimizer retargeting is
less likely to be missed between snapshots. Azimuth checks distinguish a native optimizer's
180-degree target/speed reversal from additional physical hunting; a full module
revolution remains a failure. These are software simulation tolerances.

For input/update-rate diagnosis, see [the simulation latency investigation](docs/simulation-latency.md).
`FRC_SIM_GUI=1` adds SimGUI to the websocket launcher. `FRC_SIM_LATENCY_PROBE=1`
enables raw-HAL, cached-DS, command and publisher timing probes. Desktop simulation
emits DS packet notifications once per loop so fresh raw joystick values do not
wait for a GUI-driven packet refresh.
Set `FRC_SIM_DS_PACKET_CLOCK=0` to compare the original producer-driven packet cadence.
Linux GUI simulation defaults `vblank_mode=0` in both launch paths: SimGUI polls
physical controllers on its render thread, and covered-window swap synchronization
was observed to reduce input updates to 1 Hz. An explicit `vblank_mode` overrides
the default. `scripts/sim_observe_input.py` records physical-stick cadence without
injecting input; see the investigation for measured before/after results.

In desktop Teleop, **Back/View** resets the simulation scene (body upright,
pose, wheel state and fuel). **Start/Menu** resets only the driver translation frame; it preserves the absolute
field pose used by hub aiming, vision and autonomous paths.
`/Simulation/RobotTruth` reports the robot model’s floor-level origin (settled
Z approximately zero) and actual roll/pitch; use that `Pose3d` in
AdvantageScope's 3D view to see tipping. The 2D drive pose cannot show it.
See [the rotation investigation](docs/simulation-rotation.md) for the observed
wheel-speed/heading mismatch and strengthened physical-turn check.
The unshifted physics body pose is published separately at
`/Simulation/PhysicsBodyTruth` (0.0762 m at the configured fixed module/body
geometry on flat ground). Physics validation scripts use that raw pose. The model-origin
offset follows chassis rotation and preserves bump/airborne height changes.

The desktop Full3D robot now uses [configurable chassis clearance and compressible pool-noodle bumpers](docs/compliant-bumper-contact.md) through a local, unpublished development package.

The swerve modules are hard mounted. The rigid wheel-contact model has no
vertical springs, dampers, suspension travel or load-dependent sag.
The previous MecoSimulation 2026.0.10 release provides rigid hard-mounted wheel contact, hollow bumper
contacts and load-dependent azimuth dynamics. Legacy suspension fields remain
unused. See [the rigid-contact model](docs/rigid-wheel-contact.md),
[bumper contact](docs/bumper-contact.md) and its
[bump lift validation](artifacts/bump-contact-validation-2026.0.10.json), and
[azimuth load](docs/simulation-azimuth-load.md). Jolt headers, compilation and
Debug/Release ABI settings belong entirely to the Simulation package; robot builds
do not fetch or compile Jolt source.
