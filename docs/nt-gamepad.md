# Simulation NetworkTables gamepad

The axes-only NT4 adapter is opt-in for desktop simulation: set `FRC_SIM_NT_GAMEPAD=1` before starting the simulator. Without that exact value, teleop continues to use the Driver Station controller. The adapter is compiled out of the roboRIO target. Buttons and command bindings always remain on Driver Station.

Publish `/Input/Gamepad` as a NetworkTables `double[]` with this layout:

`[1, sequence, sourceNTtimestampUs, leftY, leftX, rightX]`

Use a monotonically increasing nonnegative integer sequence and finite axes in `[-1, 1]`. `sourceNTtimestampUs` is the synchronized NT clock in microseconds. Set the NT publisher timestamp to the same `nt::Now()` value. The robot subscribes at a requested 5 ms update period and consumes the latest value on its normal robot loop. A sample is valid only when its sequence increases, the embedded source time and NT publisher server time are fresh within 50 ms, the two timestamps agree within 20 ms, and all axes are valid. Re-reading the last accepted sequence holds its axes only while fresh. Invalid, stale, malformed, or regressed input commands zero translation and rotation; it never falls back to Driver Station axes while the adapter is enabled.

Driver Station remains authoritative for enabled/teleop/E-stop state. Teleop drivetrain output is zero when disabled, stopped, or the NT sample is invalid. The adapter changes only the three axes; Driver Station buttons and bindings are unchanged. NetworkTables polling does not make the robot command loop run faster.

For comparison, `/Simulation/NTGamepad/Consumed` publishes `[sequence, sourceNTtimestampUs, consumeNTUs, commandX_mps, commandY_mps, commandOmega_radps, accepted]` once per `DriveTeleop` execution. `/Simulation/NTGamepad/DriverStationCommand` publishes `[consumeNTUs, rawLeftY, rawLeftX, rawRightX, commandX_mps, commandY_mps, commandOmega_radps]` as a Driver Station baseline. The baseline contains no separate controller-device timestamp, so it cannot establish physical USB input latency. SmartDashboard status entries report adapter enabled state, acceptance, age, and sequence.


## Run the laptop comparison

Build desktop release targets with the installed WPILib JDK:

```sh
JAVA_HOME=/home/brian/wpilib/2026/jdk ./gradlew installFrcUserProgramLinuxx86-64ReleaseExecutable installFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe --max-workers=2
```

Start the headless simulator in one terminal. Use `FRC_SIM_NT_GAMEPAD=0` for the DS baseline and `=1` for NT axes. Stop the first simulator before starting the other; keep one owner of ports 3300 and 5810.

```sh
FRC_SIM_NT_PORT=5810 FRC_SIM_LATENCY_PROBE=1 FRC_SIM_NT_GAMEPAD=0 scripts/sim_headless.sh
```

In another terminal, run each load case with the matching mode (`ds` or `nt`):

```sh
/home/brian/.cache/rebuilt-latency-venv/bin/python scripts/nt_gamepad_compare.py --mode ds --load none --seconds 8 --output artifacts/nt-gamepad/ds-none.json
/home/brian/.cache/rebuilt-latency-venv/bin/python scripts/nt_gamepad_compare.py --mode ds --load vision --seconds 8 --output artifacts/nt-gamepad/ds-vision.json
```

The Python environment needs `pyntcore==2026.2.2` and `websockets`. The script only connects to localhost and disables/zeros the simulated input in cleanup. Verify that the listeners belong to the simulator you started: the bundled HAL websocket server binds to `0.0.0.0:3300`, while this robot's configured NT server binds to `127.0.0.1:5810`.

The NT publisher embeds synchronized server time in its payload but supplies client-local time to `Publisher.set`, because NTCore performs the timestamp conversion. Publisher sequences are seeded from server time to remain increasing across script restarts while the same robot process runs. A different publisher must likewise avoid sequence regression.

## Measured comparison

Eight seconds and 110 simulated input transitions per case, using the full desktop robot at its unchanged 20 ms period. The latency endpoint is entry into the real teleop drive-command path, after input shaping, not physical motor response. Input transitions are spaced 73 ms apart to sweep relative scheduling phase. The scheduled-transition metric includes waiting for the next 20 ms DS-style HAL write or 5 ms NT sample.

| Input | Extra synthetic NT traffic | Median ms | p95 ms | p99 ms |
|---|---|---:|---:|---:|
| DS-style HAL | Off | 22.18 | 31.14 | 32.09 |
| DS-style HAL | On | 20.80 | 29.84 | 30.38 |
| NT axes | Off | 11.48 | 22.06 | 24.91 |
| NT axes | On | 13.61 | 23.20 | 24.17 |

The synthetic load publishes compact results at QuestNav-like 120 Hz, two gamepiece-camera-like 60 Hz streams, and one AprilTag-like 60 Hz stream. These are additional NT topics; they are not real headset/camera pipelines and do not exercise vision inference CPU work. Small differences between load cases are host scheduling/phase effects, not evidence that traffic improves latency. The baseline uses simulated DS/HAL writes, not the physical Driver Station UDP protocol or controller USB stack. No radio, real CAN or mechanical latency is included. All four cases passed source-rate, robot-period, telemetry and command-response checks. Raw traces and summary are under `artifacts/nt-gamepad/`.

For the complete optimized profile, including the 2 ms scheduler and separately scheduled mechanism/physics updates, see [frequency results](frequency-results.md). The earlier table above remains the original 20 ms-loop comparison.
