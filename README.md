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
and the matching-year [MecoSimulation 2026.0.8 vendordep](https://meco-robotics.github.io/MecoSimulation-cpp/MecoSimulation.json).
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

For headless Linux validation, start `./scripts/sim_headless.sh` in one terminal after building. In another terminal:

```sh
python3 -m venv /tmp/frc-cpp-sim-venv
/tmp/frc-cpp-sim-venv/bin/pip install 'pyntcore>=2026,<2027' 'robotpy-wpimath>=2026,<2027' 'websockets>=14,<17'
/tmp/frc-cpp-sim-venv/bin/python scripts/sim_smoke.py --max-speed 5.24 --reset-button 8 --reset-enabled --output artifacts/season-drive-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_mechanism_smoke.py --output artifacts/season-mechanism-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_auto_smoke.py --output artifacts/season-auto-smoke.json
```

These scripts use localhost HAL simulation and leave the simulator disabled. Before constructing the simulated drivetrain, the code derives a simulation-only steering P gain from Jolt's output-side azimuth inertia, steer ratio, motor back-EMF, current limit, and a 0.15 s nominal dominant settling target. A small derivative term (`kD = kP × 0.02 s`) adds feedback lead for the asynchronous Phoenix transport; motor back-EMF remains part of the plant damping. This is a linear model estimate; voltage, current, and simulation-step limits affect large steps. Phoenix firmware computes voltages from Jolt feedback; its built-in `DCMotorSim` plants are not stepped. The code derives PathPlanner translation and rotation gains from motor-limited chassis acceleration, mass, yaw MOI, module locations, and 8 cm/0.5 rad stopping-error scales. These simulation gains do not tune or change the real-robot gains. The drive smoke checks small-stick response, measured and target module azimuth travel at small and medium inputs, 3D chassis floor support, direction, speed tracking, pose change, and disable behavior.

The desktop-only compatibility bridge in `src/main/cpp/simulation/DriveHardwareSimulation.cpp` replaces the two hardware bridge methods from MecoSimulation 2026.0.8. It removes the competing Phoenix physics update; the same fix is committed upstream at `67e0a35`. Keep both methods together because they replace one static archive object. Remove this compatibility translation unit when adopting a Simulation release containing the fix. Scene resets reconcile odometry after five robot loops, allowing asynchronous sensor feedback to arrive without blocking the scheduler. Any autonomous pose seed and subsequent motion in that frame are preserved. This one-time correction is limited to scene reset; driving never continuously copies truth into odometry.

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

Pilot controls: left stick drives translation, right X rotates, Start resets heading, A aims while retaining translation, right trigger acquires, left trigger ejects, right bumper agitates and feeds after flywheel readiness. X selects hub/calibration, Y ferry and B idle. Copilot X/Y/B share presets; A selects trench; POV controls the intake. Actions are gated to enabled teleop. Presets latch until replaced; feed stops on cancellation.

Shooting calculations preserve all eight calibration rows, shooter exit/yaw, alliance/tag target selection, ferry trajectory exclusion, readiness ordering and volley latch/debounce. Calibration dashboard values are bounded. Autonomous includes copied PathPlanner assets, named mechanism commands, a stationary vision-initialized shot and native Choreo `LeftBlueBump` followed by a shot. Marker shots do not claim drivetrain ownership; they check the path's current heading rather than overriding the path follower.

Absolute vision comes from Limelight; PhotonVision is available as a reusable base adapter. Stationary autonomous shooting requires a recently accepted field pose; camera heartbeat alone is insufficient. Path-following autos seed their own field pose.

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
pose, wheel state and fuel). **Start/Menu** still only resets driver heading.
`/Simulation/RobotTruth` now includes actual roll/pitch; use that `Pose3d` in
AdvantageScope's 3D view to see tipping. The 2D drive pose cannot show it.
See [the rotation investigation](docs/simulation-rotation.md) for the observed
wheel-speed/heading mismatch and strengthened physical-turn check.
