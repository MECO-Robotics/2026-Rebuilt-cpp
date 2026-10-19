# Native C++ port status

## Implemented

- Reusable base drive, motor mechanisms, calibration, software profiles/PID/feedforward, physical soft limits, digital input, command cleanup and SysId.
- Native CTRE four-module swerve with season hardware mapping, fused CANcoder configuration, independent native odometry and PathPlanner GUI robot configuration.
- Season intake/shooter wiring, presets, bounded calibration, shooting/ferry/target/readiness algorithms, volley feed latch, native Choreo and autonomous assets.
- Limelight and reusable Photon pose adapters; WPILib DataLog/NetworkTables logging.
- Native ideal fuel collection, hopper packing, staggered launch, gravity flight, hub score and rolling respawn model, with deterministic regression tests.
- Test-only diagnostic runner with actuator ownership, operator start/abort, safety interlocks, stable-pass stages, prerequisite skips and JSON reports.

## Framework replacements and remaining differences

AdvantageKit is Java-only in the pinned dependency set. Native WPILib DataLog/NetworkTables replaces its logging; Java AdvantageKit logs are not replayed. Replay IO is injectable for tests, without an AdvantageKit reader. Java URCL and custom AD* telemetry are absent.

Phoenix native swerve replaces the Java base's custom module/odometry/setpoint implementation. Optional base SparkMax swerve and NavX combinations are not included. Native mechanism controllers use voltage PID/feedforward rather than vendor onboard velocity/motion-profile loops. The inertia-only simulated hood/rack explicitly use zero gravity feedforward because their plant has no gravitational load; real source gravity feedforward is retained. Adaptive integration handles the high-ratio hood without Euler instability. Hardware gains therefore require commissioning; matching source values is not hardware validation. Followers receive coordinated voltage requests, with leader telemetry. External joint encoder data is supplied through calibrated callbacks.

QuestNav tracking/relocalization and its initialization state machine are not ported; the season container uses accepted Limelight field observations directly. Java's MegaTag1/multi-tag plus MegaTag2 single-tag fallback and reference-pose single-tag filter are not identical to this native adapter. Camera mounting values are copied from source and remain hardware calibration inputs.

The native base drive simulator uses Phoenix's ideal no-slip model. Motor simulation uses simple output inertia. This does not reproduce MapleSim contact physics, detailed CAD mechanism visualization, arm/elevator gravity loading, or physical field acceptance. The replacement fuel model uses a deterministic neutral-zone grid and ideal gravity trajectories; it has no robot/structure contacts, drag, backspin or flywheel load coupling. Rack wall-contact compliance/hysteresis from the Java intake has not been ported.

The native diagnostic runner is a bounded replacement, not the Java seventeen-class system-check suite. It does not provide the original port5806 browser dashboard, per-device CAN topology audit, HTML/CSV reporting, detailed controller configuration-status or fault-injection matrix. No physical system checks have been run. Start controls must be used only after reviewing the configured stages and arranging the required unloaded setup.

## Validation

The upstream base was fully built for desktop and roboRIO, debug and release, with all nine mechanism tests passing in both configurations, and run as a real native simulator before season integration resumed. Its saved `artifacts/sim-smoke.json` verifies forward/reverse translation, rotation, tracking within10% of the requested speeds, pose changes and disabled stationary behavior. That evidence belongs to the base configuration, not this season robot.

Season desktop and roboRIO debug/release builds pass; all 29 native tests pass in both test configurations. Live native drive, mechanism-binding/output-cleanup, and Choreo autonomous smoke checks pass. Choreo endpoint error was approximately 0.346 m against the test's 0.4 m acceptance bound; this is an ideal-model regression bound, not a hardware accuracy claim. Exact build/test/runtime results are recorded in `artifacts/validation.json` and the linked evidence files. A build or numerical regression is not commissioning evidence. No roboRIO deployment or hardware motion was performed.
