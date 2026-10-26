# Simulation input latency investigation

The reported symptom was a green requested-chassis-speed arrow updating slowly
in AdvantageScope while SimGUI's joystick axes appeared current. Frame rate and
measured chassis acceleration are separate from this requested-speed signal.

## Physical-controller result and working fix

The packet-clock change below did **not** resolve the user's physical-controller
symptom. A passive 30-second capture while the stick continuously rotated showed
raw HAL, cached DS and command-read values changing about once per second
(median interval 1000 ms). The robot still produced 1496 loop probes, and maximum
scheduler execution was 1.579 ms. Requested speeds changed only 31 times.

Restarting the same simulator with `vblank_mode=0` resolved the symptom; the user
confirmed prompt requested-arrow updates with AdvantageScope covering SimGUI.
The subsequent physical-controller capture had a 20 ms median raw-HAL change
interval, 1028 requested-speed samples, and maximum scheduler execution of
1.491 ms. The recording includes an initial stationary interval; its longest
unchanged interval is not a latency measurement. These are observed input-change
intervals, not USB-to-display latency measurements.

WPILib 2026.2.1 polls physical controllers and copies them into HAL in
`DriverStationExecute()`, an early GUI-frame callback. Its
[OpenGL renderer](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/wpigui/src/main/native/opengl3/wpigui_opengl3.cpp)
enables vsync and swaps buffers on that thread. Disabling Mesa swap synchronization
removes the observed covered-window input stall on this Linux/Xwayland host.
This is a SimGUI input/render-thread coupling, not AdvantageScope polling or
scheduler starvation. An attached native stack trace was unavailable, so the
exact driver-internal wait was not measured.

Both `./gradlew simulateNative` on Linux and `FRC_SIM_GUI=1
./scripts/sim_headless.sh` now default `vblank_mode` to `0`. An explicitly supplied
value overrides this default. The GUI's configured frame limiter remains in
place; the change does not alter the 20 ms robot period or roboRIO settings.

Evidence: `artifacts/physical-stick-{packet-clock,no-vsync}.json` and
`artifacts/physical-stick-cadence-summary.json`. To passively repeat a physical
stick capture, enable Teleop and run:

```sh
python scripts/sim_observe_input.py --duration 30 --output artifacts/physical-repeat.json
```

This observer sends no joystick or DS-state writes. The automated websocket
experiments below bypass physical-controller polling and therefore did not
establish a fix for this symptom.

## Measured boundaries

Thirty-six alternating stick steps were measured in each configuration, with
AdvantageScope connected. The GUI run loaded WPILib 2026.2.1's actual SimGUI and
HAL websocket extensions. Source timestamps were converted through NT4 clock
synchronization; send times precede the websocket send. These are software
measurements, not display/pixel or USB-controller latency measurements.

| Boundary, with SimGUI loaded | Median | P95 | Maximum |
| --- | ---: | ---: | ---: |
| Input send to command reading the stick | 10.6 ms | 18.9 ms | 19.8 ms |
| Input send to requested-speed telemetry snapshot | 30.5 ms | 38.8 ms | 39.7 ms |
| Input send to a 20 ms NT client's requested-speed receipt | 43.3 ms | 52.0 ms | 58.2 ms |
| Scheduler execution | 0.210 ms | 0.453 ms | 1.721 ms |
| Telemetry dispatcher wait, representative queued probe | 0.008 ms | 0.023 ms | 0.830 ms |

The loop interval was 20.0 ms median and 20.09 ms maximum. There is a real
additional loop in requested-speed telemetry: subsystem `Periodic()` captures
its snapshot before the scheduler executes the joystick command. This accounts
for about 20 ms, not a second. No scheduler starvation or publisher backlog was
reproduced in these runs.

Those step tests explicitly sent Driver Station new-data notifications, so they
could not exclude a problem before Driver Station input ingestion.

## Raw HAL data is not a Driver Station packet

A second test sent a continuously varying raw HAL joystick axis while omitting
new-data notifications after the initial enabled packet. Raw HAL values changed
at about 46 Hz, but the robot's cached Driver Station values changed at only
0.28 Hz. With explicit packet notifications, cached values changed at 50 Hz.
This reproduces a stale-input boundary that the first test bypassed. It does
not establish why a particular GUI session failed to emit timely packets.

The pinned HAL implementation copies joystick data into its DS cache in
`HALSIM_NotifyDriverStationNewData()`. `DriverStation::RefreshData()` consumes
that cache; it does not copy arbitrary raw joystick writes into a new packet.
WPILib's SimGUI normally notifies at a nominal 20 ms cadence, conditional on its
DS state, timing and GUI execution. See the
[pinned DriverStationGui source](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp).

The season simulator now emits a nonblocking HAL packet notification at the end
of `Robot::SimulationPeriodic()`. The next loop performs its normal DS refresh
before mode handling and command scheduling. This decouples packet ingestion
from GUI redraw cadence. It preserves the raw attached/enabled/e-stop state and
does not enable the robot, advance physics again, or change the scheduler rate.
The code is excluded from roboRIO builds. Set `FRC_SIM_DS_PACKET_CLOCK=0`
to restore producer-owned notifications for an A/B comparison or an external
Driver Station packet-timing test.

Before/after records are in `artifacts/driverstation-input-cadence-{before,after}.json`.
The regression requires raw and cached input changes above 25 Hz and median
raw/cached disagreement below 0.08 for the 1 Hz, 0.4-amplitude stimulus. A full
change count is not a USB polling-rate measurement.

## Direct requested-speed A/B check

The same release binary, with SimGUI and AdvantageScope connected, received 12
alternating sustained raw-only stick steps in each mode (about 1.15 seconds per
step). No DS notifications were sent by the test during a step.

| Packet clock | Steps observed during their hold window | Requested source latency | 20 ms NT client receipt |
| --- | ---: | ---: | ---: |
| Off | 1 / 12 | 665 ms for the one observed step | 683 ms |
| On | 12 / 12 | 82 ms median; 131 ms maximum | 87 ms median; 140 ms maximum |

Missing off-mode steps are censored observations, not zero latency and not part
of a fabricated average. This reproduces a substantial input-delivery failure
and demonstrates the fix at the requested-speed signal. The on-mode figures
include contention with SimGUI's joystick writes and the next-loop DS refresh.
They do not measure the physical controller or rendered arrow. Records are
`artifacts/requested-latency-clock-{off,on}.json`; their binary hashes match.

The final full Gradle build passed desktop and roboRIO Debug/Release targets,
with 21 season tests passing in each desktop test configuration. The separate
raw/cached integration regression fails before the clock and passes after it.

## AdvantageScope connection

The installed application was AdvantageScope 26.0.0. Live server metadata
reported `periodic = 0.02` and `all = true` for its actual requested-speed and
module-state subscriptions. During a circular-input stimulus, Linux TCP counters
on that exact AdvantageScope connection showed frequent data delivery, not a
one-second send interval. `ss` observations give a lower bound because multiple
packets can arrive between observations. This is not a measurement of the
application's parser, timeline selection, or rendered frames.

Metadata and TCP observations are retained in
`artifacts/advantagescope-subscriptions.json` and
`artifacts/advantagescope-wire-cadence.json`. The user's observed display symptom
must not be called a confirmed renderer bottleneck merely from these results.

## Reproduce

After a normal build, launch the diagnostic simulator:

```sh
FRC_SIM_GUI=1 FRC_SIM_LATENCY_PROBE=1 ./scripts/sim_headless.sh
```

Omit `FRC_SIM_GUI=1` for the headless comparison. The launcher snapshots the
binary/libraries and loads the same HAL websocket extension in either mode.
`FRC_SIM_LATENCY_PROBE` is opt-in; ordinary runs publish no latency probes.

Use an isolated Python environment with `pyntcore`, `robotpy-wpimath` (2026),
`websockets` (14–16), and `msgpack` for the socket-cadence helper:

```sh
python scripts/sim_latency.py --output artifacts/simulation-latency-repeat.json
python scripts/sim_latency.py --raw-only --steps 12 --output artifacts/raw-input-repeat.json
python scripts/sim_input_cadence.py --output artifacts/input-cadence-repeat.json
python scripts/advantagescope_cadence.py --output artifacts/AS-cadence-repeat.json
```

`--raw-only` holds each stick step with repeated raw writes at about 100 Hz,
without sending DS notifications during the step. Repeating writes matters when
SimGUI also owns a joystick slot; one websocket write can be overwritten before
the next robot loop. Repeat with the packet clock disabled to isolate ingestion.

These scripts drive only localhost HAL and disable it during cleanup. The last
helper requires one connected AdvantageScope client and Linux `ss`.

`/Diagnostics/Latency/RobotLoop` contains: loop-start NT microseconds,
scheduler-end NT microseconds, cached forward stick, command-read forward stick,
command-read NT microseconds, FPGA microseconds, raw HAL forward stick, cached DS
attached, cached enabled. `Background` appends the worker publication timestamp.
Both arrays capture data by value; the worker does not read robot state.
