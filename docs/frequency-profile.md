# Configurable control profile

The checked-in profile enables a 2 ms command scheduler on desktop and roboRIO 2. `TimedRobot` retains its 20 ms callbacks for match context, telemetry and simulation physics. The scheduler callback runs on the same main thread; it does not create a worker thread. This is a requested cadence, not a hard deadline guarantee.

Edit `src/main/deploy/fast-control.json` before deployment. The robot loads it once at startup; changes require a robot-program restart. Unknown keys, invalid types and out-of-range values stop startup with an error. A missing file uses the checked-in defaults. The ordinary Gradle build uses matching public Base 2026.2.12 and Simulation 2026.0.17 packages; no local Maven override is needed.

| Setting | Default | Meaning |
|---|---:|---|
| `enabled` | `true` | Fast scheduler, mechanism and CAN profile; false restores 20 ms scheduling/default CAN rates |
| `periodMs` | 2 | Command scheduler period, 1–20 ms |
| `ntGamepadEnabled` | `true` | Permit NT axes, independently of fast scheduling |
| `driverInput` | `"nt"` | `nt` or `driverstation`; buttons always use DS |
| `ntPeriodMs` | 5 | Requested gamepad subscription period |
| `ntStaleMs` | 50 | Maximum accepted input age |
| `sparkControlMs` | 10 | SPARK mechanism control period |
| `talonFdControlMs` / `talonClassicControlMs` | 4 / 10 | Kraken mechanism control period for detected bus type |
| `sparkStatusHz` | 100 | SPARK position/velocity request |
| `talonFdStatusHz` / `talonClassicStatusHz` | 250 / 100 | Kraken position/velocity request |
| `odometryFdHz` / `odometryClassicHz` | 500 / 100 | Drivetrain odometry request |
| `diagnosticHz` | 4 | Noncritical mechanism diagnostics; fault and follower-critical frames retain protected rates |
| `questPeriodMs` | 5 | QuestNav NT subscription period, 5–1000 ms |
| `detectionPeriodMs` | 5 | Gamepiece detection NT subscription request |

Mechanism control periods must be integer multiples of `periodMs`. CAN FD selection is detected for each configured bus, with classic CAN fallbacks; bus names, motor IDs, Pigeon and existing wiring are retained. These are requested CAN rates, not measured physical delivery or utilization. Controller-internal PID frequency is separate. QuestNav defaults to 5 ms transport; its measurement rate remains limited by the headset. Camera publishers must also use the faster transport, for example through `scripts/perception_publisher.py`; changing the subscriber does not accelerate inference.

For the original baseline, set `enabled: false`, `driverInput: "driverstation"` and `ntGamepadEnabled: false`, and `detectionPeriodMs: 100`; retain the default period fields. For a 4 ms scheduler, also choose mechanism periods divisible by 4 (for example SPARK/classic 12 ms, FD 4 ms). Desktop benchmark scripts can override fast scheduling with `FRC_FAST_CONTROL=0|1` and NT selection with `FRC_SIM_NT_GAMEPAD=0|1`; these are benchmark overrides. On hardware use the deploy configuration.

With NT input selected, the USB-controller producer must run on the laptop. Missing, invalid or stale NT input produces zero axes with no automatic DS-axis fallback. Driver Station continues to own buttons, enable/disable and E-stop. See [controller setup](nt-gamepad.md).

Drive field-relative discretization uses the configured scheduler period. Desktop drivetrain and mechanism physics still advance exactly once per 20 ms simulation callback. The desktop measurements in [frequency results](frequency-results.md) are historical laptop evidence; roboRIO timing and real CAN throughput require separate measurement.

`/Simulation/FastControl/Samples` publishes one double array per fast callback:

`[sequence, scheduledFpgaUs, actualStartFpgaUs, finishFpgaUs, workUs, latenessUs, enabled]`

The array separates callback work from scheduling interval and lateness. It is diagnostic output and contributes some NT traffic. A 2 ms average interval or short desktop run does not establish hard real-time behavior, physical drivetrain response, CAN capacity, or safe robot operation.

The NT gamepad adapter has an independent input selection. Its `/Simulation/NTGamepad/Consumed` telemetry carries the NT source and consumption timestamps for input-to-command age; compare those in the NT clock domain rather than subtracting them from FPGA timestamps.

See [measured results and reproducible local build](frequency-results.md) for the baseline comparisons, hardware-validation limits and matching Base/Simulation development packages.
