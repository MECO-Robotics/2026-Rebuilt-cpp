# 2026 REBUILT — C++

MECO Robotics team 8324's native WPILib 2026 C++ robot project. This repository is a GitHub fork of [2026_Base_Bot-cpp](https://github.com/MECO-Robotics/2026_Base_Bot-cpp), created after the base was ported. The base's validated native-simulation changes are included in this history.

The original Java projects are preserved in sibling folders. See [the port contract](docs/port-contract.md) for source commits, units and validation boundaries.

For architecture-aware Codex assistance, use the repository skill [`frc-base-project`](.agents/skills/frc-base-project/SKILL.md), or invoke `$frc-base-project`. Test guidance lives in `AGENTS.md`.

For device configuration, commissioning, and diagnosis, use [`frc-hardware`](.agents/skills/frc-hardware/SKILL.md) or invoke `$frc-hardware`.

## Build and simulate

Install WPILib 2026.2.1 with its C++ toolchains and native vendor dependencies:

```sh
./gradlew build --max-workers=2
./gradlew simulateNative
```

The build includes desktop and roboRIO debug/release executables and GoogleTest regressions. Deployment is `./gradlew deploy` and writes to team 8324's roboRIO. This port has not been deployed or commissioned on hardware.

For headless Linux validation, start `./scripts/sim_headless.sh` in one terminal after building. In another terminal:

```sh
python3 -m venv /tmp/frc-cpp-sim-venv
/tmp/frc-cpp-sim-venv/bin/pip install 'pyntcore>=2026,<2027' 'websockets>=14,<17'
/tmp/frc-cpp-sim-venv/bin/python scripts/sim_smoke.py --max-speed 5.24 --reset-button 8 --reset-enabled --output artifacts/season-drive-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_mechanism_smoke.py --output artifacts/season-mechanism-smoke.json
/tmp/frc-cpp-sim-venv/bin/python scripts/season_auto_smoke.py --output artifacts/season-auto-smoke.json
```

These scripts use localhost HAL simulation and leave the simulator disabled. They verify drive response, real button bindings and motor outputs, and the Choreo chooser/trajectory endpoint. Simulated rollers can coast after voltage is removed; cancellation checks applied output rather than assuming an immediate mechanical stop. Fuel collection and projectile telemetry appears under `FieldSimulation/`.

## Robot behavior

The season container supplies the Java robot's CAN IDs, bus names, gear ratios, encoder offsets, camera transform and mechanism presets. Linear rack positions are metres; hood positions are rotations; flywheel rates are rotations/second. The rack's Java motor-rotations-per-metre conversion is preserved through a physical 9:1 output ratio and drum circumference.

Pilot controls: left stick drives translation, right X rotates, Start resets heading, A aims while retaining translation, right trigger acquires, left trigger ejects, right bumper agitates and feeds after flywheel readiness. X selects hub/calibration, Y ferry and B idle. Copilot X/Y/B share presets; A selects trench; POV controls the intake. Actions are gated to enabled teleop. Presets latch until replaced; feed stops on cancellation.

Shooting calculations preserve all eight calibration rows, shooter exit/yaw, alliance/tag target selection, ferry trajectory exclusion, readiness ordering and volley latch/debounce. Calibration dashboard values are bounded. Autonomous includes copied PathPlanner assets, named mechanism commands, a stationary vision-initialized shot and native Choreo `LeftBlueBump` followed by a shot. Marker shots do not claim drivetrain ownership; they check the path's current heading rather than overriding the path follower.

Absolute vision comes from Limelight; PhotonVision is available as a reusable base adapter. Stationary autonomous shooting requires a recently accepted field pose; camera heartbeat alone is insufficient. Path-following autos seed their own field pose.

## Diagnostics

In enabled Test mode, use SmartDashboard `SystemCheck/Armed`, `SystemCheck/SetupConfirmed`, then `SystemCheck/Start`; `SystemCheck/Abort` cancels. The runner reserves all mechanism requirements, checks DS/FMS/e-stop/brownout/voltage interlocks, counts down, evaluates connectivity and bounded mechanism stages, and neutralizes on interruption or mode loss. Reports are JSON on the dashboard and `/home/lvuser/systemcheck-latest.json` (local file in desktop simulation). Read `docs/cpp-port-status.md` before using this replacement for the Java pit-check system.

## Upstream updates

```sh
git fetch upstream
git merge upstream/main
```

`origin` is the season fork; `upstream` is the C++ base. Season code lives in `season/` and `RobotContainer`; small base configuration extensions in this fork supply the original Phoenix/PathPlanner season setup.

## Migration finalization

The `codex/finalize-cpp-migration` branch merges generic changes from the C++ base and retains season hardware IDs, offsets, gearing, source gains and autonomous assets. See [the finalization review](docs/cpp-finalization.md), [current port status](docs/cpp-port-status.md), and `artifacts/finalization-validation.json` for current evidence and remaining gaps. Earlier validation artifacts remain historical evidence.

QuestNav now uses the pinned 2026 protobuf transport, source mounting transform and covariance, freshness/field readiness checks, version/battery diagnostics and absolute-vision fallback. Local field resets synchronize operator, AutoBuilder and Choreo resets. There is no physical headset or calibration evidence, headset reset ACK protocol, or automatic Quest noise/drift plant.

The season requests 250 Hz native Phoenix odometry/control on `MECO CANIvore`. Commands retain exclusive ownership at 50 Hz, publishing targets to native-rate heading feedback. Native swerve physics runs on a separate 4 ms notifier. Pose/speed telemetry is capped at 10 Hz, bus diagnostics at 1 Hz. Moving targets expire after 100 ms without refresh, and disabled ticks neutralize/invalidate targets. This preserves Phoenix ownership; Phoenix already repeated native requests before this work.

Each mechanism opts into disabled-only tuning at `<name>/Gains/Values`, with its ten-field order at `<name>/Gains/Fields` and acceptance status at `<name>/Gains/Status`. The whole candidate array must validate; enabled updates are locked, physical limits are excluded, and values are not persisted across reboot. Native versioned `<name>/MotorInputs` logs include disabled and fault inputs. MotorIO replay and model-derived feedforward pretuning are reusable base facilities; these do not constitute whole-robot replay or automatic hardware calibration.

For the additional disabled-only Quest transport/fallback fixture, run the local headless simulator and `python scripts/questnav_smoke.py --output artifacts/finalization-questnav-smoke.json` in the documented Python environment. It injects synthetic NetworkTables sensor data only; stop/restart the simulator before other smoke checks so those fixtures cannot contaminate their measurements.
