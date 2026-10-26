# Native C++ port status

## Implemented

- Shared drive, mechanism, IO, characterization and vision implementations are consumed from the pinned `MecoBase` WPILib vendordep; season hardware mappings and behavior remain local.
- Reusable base drive, motor mechanisms, calibration, software profiles/PID/feedforward, physical soft limits, digital input, command cleanup and SysId.
- Native CTRE four-module swerve with season hardware mapping, remote CANcoder feedback, independent native odometry and PathPlanner GUI robot configuration. The released vendordep does not expose the former fused-feedback switch.
- Season intake/shooter wiring, presets, bounded calibration, shooting/ferry/target/readiness algorithms, volley feed latch, native Choreo and autonomous assets.
- Limelight and reusable Photon pose adapters; WPILib DataLog/NetworkTables logging.
- Native ideal fuel collection, hopper packing, staggered launch, gravity flight, hub score and rolling respawn model, with deterministic regression tests.
- Test-only diagnostic runner with actuator ownership, operator start/abort, safety interlocks, stable-pass stages, prerequisite skips and JSON reports.

## Framework replacements and remaining differences

AdvantageKit is Java-only in the pinned dependency set. Native WPILib DataLog/NetworkTables replaces its logging; Java AdvantageKit logs are not replayed. Replay IO is injectable for tests, without an AdvantageKit reader. Java URCL and custom AD* telemetry are absent.

Phoenix native swerve replaces the Java base's custom module/odometry/setpoint implementation. Optional base SparkMax swerve and NavX combinations are not included. Native mechanism controllers use voltage PID/feedforward rather than vendor onboard velocity/motion-profile loops. The inertia-only simulated hood/rack explicitly use zero gravity feedforward because their plant has no gravitational load; real source gravity feedforward is retained. Adaptive integration handles the high-ratio hood without Euler instability. Hardware gains therefore require commissioning; matching source values is not hardware validation. Followers receive coordinated voltage requests, with leader telemetry. External joint encoder data is supplied through calibrated callbacks.

QuestNav tracking/relocalization and its initialization state machine are not ported; the season container uses accepted Limelight field observations directly. Java's MegaTag1/multi-tag plus MegaTag2 single-tag fallback and reference-pose single-tag filter are not identical to this native adapter. Camera mounting values are copied from source and remain hardware calibration inputs.

The desktop drive simulator uses the optional MecoSimulation Jolt 3D adapter with the season chassis inputs: 53 kg, 6.883 kg m² yaw MOI, 2.9 wheel coefficient of friction, 2 in wheel radius, 5.8909 drive ratio, and 55 A drive current limit. PathPlanner's GUI settings use the same values. The yaw MOI is still an estimate copied from PathPlanner; replace it with a CAD or measured value when available. HAL joystick smoke and square/circle sweeps cover small-stick, changing-direction tracking, physical-truth/odometry agreement, repeated scene reset, floor support and disable response. A local desktop compatibility bridge removes the competing Phoenix physics step from Simulation 2026.0.8; the same fix is committed upstream. See `artifacts/drivetrain-simulation-validation.json` for current evidence. These checks do not establish physical calibration. The replacement fuel model uses a deterministic neutral-zone grid and ideal gravity trajectories; it has no robot/structure contacts, drag, backspin or flywheel load coupling. Rack wall-contact compliance/hysteresis from the Java intake has not been ported.

The native diagnostic runner is a bounded replacement, not the Java seventeen-class system-check suite. It does not provide the original port5806 browser dashboard, per-device CAN topology audit, HTML/CSV reporting, detailed controller configuration-status or fault-injection matrix. No physical system checks have been run. Start controls must be used only after reviewing the configured stages and arranging the required unloaded setup.

## Validation

The upstream base was fully built for desktop and roboRIO, debug and release, with all nine mechanism tests passing in both configurations, and run as a real native simulator before season integration resumed. Its saved `artifacts/sim-smoke.json` verifies forward/reverse translation, rotation, tracking within10% of the requested speeds, pose changes and disabled stationary behavior. That evidence belongs to the base configuration, not this season robot.

Season desktop and roboRIO debug/release builds pass against the published `MecoBase` 2026.2.0 and `MecoSimulation` 2026.0.3 vendordeps; all 20 season tests pass in both Debug and Release. The public Maven resolution, deployed package checks, and Linux desktop drive smoke are recorded in `artifacts/public-multivendordep-validation-2026-10-25.json`; the smoke exercises the 2026 Jolt adapter through HAL and NT4. Older Choreo and mechanism smoke results predate this vendordep integration. A build or numerical regression is not commissioning evidence. No roboRIO deployment or hardware motion was performed.

On October 26, the fixed desktop bridge, steering damping and asynchronous scene
reset reconciliation passed the full desktop/roboRIO Debug/Release build and all
21 season tests in each desktop configuration. Square, circular-stick and
circular-stick-with-yaw HAL sweeps passed; worst physical/odometry pose difference
was 0.059 m and physical/encoder velocity error P95 was below 0.072 m/s.
The Choreo endpoint check passed within 0.030 m while preserving its pose seed. Retained
baseline traces fail the same analysis gates. The source compatibility bridge is
local to this season project pending an upstream Simulation package release;
Linux runtime acceptance does not establish Windows runtime or hardware behavior.
