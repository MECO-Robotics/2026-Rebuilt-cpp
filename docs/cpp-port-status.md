# Native C++ port status

## Implemented

- Reusable base drive, motor mechanisms, calibration, software profiles/PID/feedforward, physical soft limits, digital input, command cleanup and SysId.
- Native CTRE four-module swerve with season hardware mapping, fused CANcoder configuration, independent native odometry and PathPlanner GUI robot configuration.
- Season intake/shooter wiring, presets, bounded calibration, shooting/ferry/target/readiness algorithms, volley feed latch, native Choreo and autonomous assets.
- Limelight, reusable Photon and read-only pinned QuestNav protobuf adapters with freshness/readiness gating, alignment and absolute fallback; WPILib DataLog/NetworkTables logging.
- Native-rate field transforms and heading feedback with bounded target handoff, stale moving-target expiry, disable invalidation and sole actuator ownership; explicit 250 Hz season CANivore configuration.
- All seven mechanisms expose validated disabled-only gain tuning and versioned MotorIO input logging. Offline native replay, deterministic multirate/latency harness and opt-in model feedforward helpers are available.
- Native ideal fuel collection, hopper packing, staggered launch, gravity flight, hub score and rolling respawn model, with deterministic regression tests.
- Test-only diagnostic runner with actuator ownership, operator start/abort, safety interlocks, stable-pass stages, prerequisite skips and JSON reports.

## Framework replacements and remaining differences

AdvantageKit is Java-only in the pinned dependency set. Native WPILib DataLog/NetworkTables replaces its logging; Java AdvantageKit logs are not replayed. Versioned native MotorIO logs can be loaded into timestamped replay IO. Full-robot configuration/commands/DriverStation chronology and external sensors must be restored separately; there is no AdvantageKit reader. Java URCL and custom AD* telemetry are absent.

Phoenix native swerve replaces the Java base's custom module/odometry/setpoint implementation. Optional base SparkMax swerve and NavX combinations are not included. Native mechanism controllers use voltage PID/feedforward rather than vendor onboard velocity/motion-profile loops. The inertia-only simulated hood/rack explicitly use zero gravity feedforward because their plant has no gravitational load; real source gravity feedforward is retained. Adaptive integration handles the high-ratio hood without Euler instability. Hardware gains therefore require commissioning; matching source values is not hardware validation. Followers receive coordinated voltage requests, with leader telemetry. External joint encoder data is supplied through calibrated callbacks.

QuestNav 2026-2.2.0 frames/device data are decoded natively and aligned using the copied season mounting transform; accepted Quest poses feed estimation, with validated absolute-camera fallback. This is a read-only wire implementation: local field resets are synchronized for operator/AutoBuilder/Choreo resets, while headset reset ACKs and Java stochastic drift simulation remain absent. Java's MegaTag1/multi-tag plus MegaTag2 single-tag fallback and reference-pose single-tag filter are not identical to this native adapter. Camera mounting values are copied from source and remain hardware calibration inputs.

The native drive simulator uses Phoenix's ideal no-slip model with a sole 4 ms plant notifier. The separate integer 250/50 Hz replay/latency harness is deterministic offline support; Phoenix's live estimator and simulation threads remain wall-clock scheduled. Fuel reset now restores its seeded RNG. Motor simulation uses simple output inertia. This does not reproduce MapleSim contact physics, detailed CAD mechanism visualization, arm/elevator gravity loading, or physical field acceptance. The replacement fuel model uses a deterministic neutral-zone grid and ideal gravity trajectories; it has no robot/structure contacts, drag, backspin or flywheel load coupling. Rack wall-contact compliance/hysteresis from the Java intake has not been ported.

The native diagnostic runner is a bounded replacement, not the Java seventeen-class system-check suite. It does not provide the original port5806 browser dashboard, per-device CAN topology audit, HTML/CSV reporting, detailed controller configuration-status or fault-injection matrix. No physical system checks have been run. Start controls must be used only after reviewing the configured stages and arranging the required unloaded setup.

## Current finalization evidence

See `docs/cpp-finalization.md` for architecture, numerical estimates and remaining gaps, and `artifacts/finalization-validation.json` for current build/test/simulator evidence. Invalid/nonfinite feedback cannot authorize feeding; finite manual volley sag behavior remains intact. Diagnostic Abort consumes a queued Start in the same tick. CAN health is sampled on the robot thread rather than the native control callback. Full browser/device-topology diagnostic parity remains open.

## Historical migration validation

The upstream base was fully built for desktop and roboRIO, debug and release, with all nine mechanism tests passing in both configurations, and run as a real native simulator before season integration resumed. Its saved `artifacts/sim-smoke.json` verifies forward/reverse translation, rotation, tracking within10% of the requested speeds, pose changes and disabled stationary behavior. That evidence belongs to the base configuration, not this season robot.

Season desktop and roboRIO debug/release builds pass; all 29 native tests pass in both test configurations. Live native drive, mechanism-binding/output-cleanup, and Choreo autonomous smoke checks pass. Choreo endpoint error was approximately 0.346 m against the test's 0.4 m acceptance bound; this is an ideal-model regression bound, not a hardware accuracy claim. Exact build/test/runtime results are recorded in `artifacts/validation.json` and the linked evidence files. A build or numerical regression is not commissioning evidence. No roboRIO deployment or hardware motion was performed.
