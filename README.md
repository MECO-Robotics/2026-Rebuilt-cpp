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
and the matching-year [MecoSimulation 2026.0.7 vendordep](https://meco-robotics.github.io/MecoSimulation-cpp/MecoSimulation.json).
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

These scripts use localhost HAL simulation and leave the simulator disabled. Before constructing the simulated drivetrain, the code derives a simulation-only steering P gain from Jolt's output-side azimuth inertia, steer ratio, motor back-EMF, current limit, and a 0.15 s dominant settling target. It uses no additional D gain because motor back-EMF already damps the azimuth plant. This is a linear model estimate; voltage, current, and simulation-step limits affect large steps. Phoenix's `.004 kg·m²` Talon rotor model remains internal to its voltage model and is not the inertia used for gain derivation. The code derives PathPlanner translation and rotation gains from motor-limited chassis acceleration, mass, yaw MOI, module locations, and 8 cm/0.5 rad stopping-error scales. These simulation gains do not tune or change the real-robot gains. The drive smoke checks small-stick response, measured and target module azimuth travel at small and medium inputs, 3D chassis floor support, direction, speed tracking, pose change, and disable behavior. The mechanism smoke checks button bindings and motor outputs; the auto smoke checks the Choreo chooser/trajectory endpoint. Simulated rollers can coast after voltage is removed; cancellation checks applied output rather than assuming an immediate mechanical stop. Fuel collection and projectile telemetry appears under `FieldSimulation/`.

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
