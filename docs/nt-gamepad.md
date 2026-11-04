# NetworkTables gamepad input

The axes-only NT4 adapter is enabled by `kEnabled` at the top of [NtGamepad.h](../src/main/include/season/drive/NtGamepad.h); desktop simulation can override the setting with `FRC_SIM_NT_GAMEPAD=1` or `=0`. Its default subscription period is 5 ms, while robot commands and mechanism control run on the standard 20 ms robot loop configured in [RobotControlConfig.h](../src/main/include/season/RobotControlConfig.h). Buttons and command bindings always remain on Driver Station.

Publish `/Input/Gamepad` as a NetworkTables `double[]` with this layout:

`[1, sequence, sourceNTtimestampUs, leftY, leftX, rightX]`

Use a monotonically increasing nonnegative integer sequence and finite axes in `[-1, 1]`. `sourceNTtimestampUs` is the synchronized NT clock in microseconds. Embed that server-time value in the payload, but pass the client-local `nt::Now()` value as the NT publisher timestamp; NTCore converts it to server time. The robot subscribes at a requested 5 ms update period and consumes the latest value on its 20 ms command cadence. A sample is valid only when its sequence increases, the embedded source time and NT publisher server time are fresh within 50 ms, the two timestamps agree within 20 ms, and all axes are valid. Re-reading the last accepted sequence holds its axes only while fresh. Invalid, stale, malformed, or regressed NT input falls back to connected, valid Driver Station joystick axes by default. Fresh valid NT input automatically takes priority again. Fallback retains the normal Driver Station packet cadence. Set `kFallbackToDriverStation` to `false` in `NtGamepad.h` to zero the axes instead. If neither input source is valid, translation and rotation are zero. `Input/ActiveSource` reports the selected source.

Driver Station remains authoritative for enabled/teleop/E-stop state. Teleop drivetrain output is zero when disabled, stopped, or neither the NT stream nor the allowed Driver Station fallback is valid. The adapter changes only the three axes; Driver Station buttons and bindings are unchanged. NetworkTables polling does not make the robot command loop run faster.

## Use a USB controller with the roboRIO

Connect a USB gamepad to the driver laptop, install the two Python packages, then run the producer below. Substitute the team number or use `--server` for a custom NT4 host. This publishes axes only; continue using the Driver Station application to enable, disable, select mode, and operate buttons.

```sh
python -m pip install 'pygame-ce>=2.5.8,<3' 'pyntcore==2026.2.2'
python scripts/nt_gamepad.py --team 8324
```

The script uses joystick 0, SDL's standardized left stick Y/X for translation, and right stick X for rotation; it publishes at 200 Hz to match the robot's requested 5 ms input period. A controller needs an SDL GameController mapping. If none is available, the script exits with instructions rather than guessing: inspect raw axes using `--print-only --raw-axes`, then pass `--raw-axes --left-y N --left-x N --right-x N`. Supplying any raw axis option also selects raw mode. Use `--help` for other overrides. Inversion and `--deadband 0.05` are configurable; deadband rescales the remaining range. `--print-only` inspects input without sending commands and needs `pygame` but not `pyntcore`. The producer keeps reading when its window is unfocused. On USB removal it stops publishing and exits; Ctrl-C also stops publishing. In either case, robot-side freshness checks select Driver Station joystick axes after the configured stale timeout (50 ms by default), or zero the axes if fallback is disabled or the DS joystick is unavailable. Reconnect and restart the script after a disconnect. Configure a controller and verify the directions with the robot safely disabled before enabling.

An optional JSON file can hold laptop-specific settings, with explicit command-line options taking precedence:

```json
{
  "team": 8324,
  "rate": 200,
  "joystick": 0,
  "raw_axes": false,
  "invert_left_y": false,
  "invert_left_x": false,
  "invert_right_x": false,
  "deadband": 0.05,
  "topic": "/Input/Gamepad"
}
```

For a device without an SDL mapping, set `"raw_axes": true` and provide its `left_y`, `left_x`, and `right_x` axis indices. Supplying raw indices in the file enables raw mode unless `raw_axes` is explicitly set to `false`.

Run it with `python scripts/nt_gamepad.py --config gamepad.json`. The producer starts only after it can connect to the configured NT4 server and obtain the synchronized server-time offset. It places the synchronized server timestamp in the payload and passes client-local time to NTCore for the publisher timestamp conversion; sequence numbers are seeded from synchronized server time so restarting the laptop script does not regress the robot's monotonic sequence check.

For comparison, `/Simulation/NTGamepad/Consumed` publishes `[sequence, sourceNTtimestampUs, consumeNTUs, commandX_mps, commandY_mps, commandOmega_radps, accepted]` once per `DriveTeleop` execution. `/Simulation/NTGamepad/DriverStationCommand` publishes `[consumeNTUs, rawLeftY, rawLeftX, rawRightX, commandX_mps, commandY_mps, commandOmega_radps]` as a Driver Station baseline. The baseline contains no separate controller-device timestamp, so it cannot establish physical USB input latency. SmartDashboard status entries report adapter enabled state, acceptance, age, and sequence.


## Run the laptop comparison

These headless simulator and comparison commands are for Linux x86-64. The USB controller publisher above also supports other platforms.

Build desktop release targets with the installed WPILib JDK. If Gradle cannot find it, set `JAVA_HOME` to your WPILib 2026 JDK directory first:

```sh
./gradlew installFrcUserProgramLinuxx86-64ReleaseExecutable installFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe --max-workers=2
```

Start the headless simulator in one terminal. Use `FRC_SIM_NT_GAMEPAD=0` for the DS baseline and `=1` for NT axes. Stop the first simulator before starting the other; keep one owner of ports 3300 and 5810.

```sh
FRC_SIM_NT_PORT=5810 FRC_SIM_LATENCY_PROBE=1 FRC_SIM_NT_GAMEPAD=0 scripts/sim_headless.sh
```

In another terminal, use a Python environment with `pyntcore==2026.2.2` and `websockets==16.1.1` installed, then run each load case with the matching mode (`ds` or `nt`):

```sh
python scripts/nt_gamepad_compare.py --mode ds --load none --seconds 8 --output artifacts/nt-gamepad/ds-none.json
python scripts/nt_gamepad_compare.py --mode ds --load vision --seconds 8 --output artifacts/nt-gamepad/ds-vision.json
```

The script only connects to localhost and disables/zeros the simulated input in cleanup. Verify that the listeners belong to the simulator you started: the bundled HAL websocket server binds to `0.0.0.0:3300`, while this robot's configured NT server binds to `127.0.0.1:5810`.

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

For historical measurements from an earlier fast-control experiment, see [frequency results](frequency-results.md). Those settings are no longer part of the active robot code.
