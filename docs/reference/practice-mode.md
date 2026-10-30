> Historical detail: this record may describe earlier versions or experiments. Start with the [current guide](../practice-mode.md).

# Simulation practice matches

The desktop robot instantiates MecoSimulation's `PracticeMatchController` with
20 seconds AUTO, a 3-second disabled scoring-assessment gap, and 140 seconds
TELEOP. The gap is separate from TELEOP's first 10-second transition shift.
No companion planner or custom GUI is required. This control interface is absent
from roboRIO builds. Desktop simulation selects offensive AutoDrive for
autonomous and LB in teleop. While disabled, use the SmartDashboard
**Start Practice Match** command to start with the SimGUI alliance station
(Blue1 if no station is selected). Alternatively,
use NT4 dashboard widgets to
publish the following topics. Commands are string **updates**, so sending `start`
again after a completed/aborted session starts another match.

| `/Sim/Practice/` topic | Type | Meaning |
|---|---|---|
| `Command` | string | `start`, `abort`, `reset` |
| `Config/AllianceStation` | string | `Red1`–`Red3`, `Blue1`–`Blue3`; default `Blue1` |
| `Config/AutoResult` | string | `R` or `B`: alliance whose hub goes inactive first; default `R` |
| `State` | string | `idle`, `autonomous`, `assessment`, `teleop`, `complete`, `aborted` |
| `RemainingSeconds` | double | Current phase countdown |
| `MatchTime` | double | Applied DS period timer; -1 when unavailable |
| `GameData` | string | Current DS game message |
| `Running` | boolean | Controller owns the practice sequence |
| `LastCommand`, `LastCommandAccepted`, `Reason` | string, boolean, string | Acknowledgement/rejection or abort reason |
| `Profile/AutoSeconds`, `Profile/AssessmentSeconds`, `Profile/TeleopSeconds` | double | Published timing profile; not configuration inputs |

Station names map to WPILib’s corresponding alliance stations; the unknown
station value is never used for an accepted configuration. Configuration is
validated and sampled on start. Start requires disabled and no
E-stop; retained commands present at construction are ignored. Game data is empty
in AUTO/assessment and delivered at TELEOP start. The existing chooser supplies
the autonomous command. No new scoring/driving strategy is selected by practice.

Manual disable during an enabled phase, mode/test/alliance/attachment takeover,
or E-stop aborts the run, including brief changes between periodic updates.
Pending commands at interruption are discarded. Only a subsequent fresh `start`
can run again. E-stop is never cleared by the controller. Completion/abort disable,
restore saved attachment/alliance/match metadata, and relinquish control. A
controller destroyed during a run also disables. Reset clears the practice
session without resetting physical pose, mechanisms, or fuel.

Use one DS control owner: do not simultaneously run another automated HAL driver
or an external Driver Station. SimGUI may display/control DS state; its manual
mode controls intentionally abort an active practice run. GUI timer coexistence
must be checked with the GUI extension loaded, separately from headless checks.

## Match context and hub timing

`/Match/` publishes `Phase`, `Time`, `TimeKnown`, `GameData`, `GameDataValid`,
`HubKnown`, `HubActive`, `BlueHubKnown`, `RedHubKnown`, `BlueHubActive`,
`RedHubActive`, `BlueNextChange`, `RedNextChange`, `InactiveFirstAlliance`,
`FirstActiveAlliance`, `HubNextChangeKnown`, and `HubNextChangeSeconds`. Unknown numeric values use
-1 and unknown activity must be interpreted with the accompanying known flag.
AutoDrive receives this same season snapshot through an injected supplier and
publishes its consumed phase/time/own-hub status, inactive-first and first-active
alliance, and own-hub next-change time under SmartDashboard/AutoDrive. The
`HubOrderKnown` and `HubNextChangeKnown` flags distinguish unknown values.
It does not read simulation practice commands.

Both hubs are active in AUTO, TELEOP transition (140–130 seconds remaining), and
endgame (30–0). Four 25-second alliance shifts begin at 130, 105, 80, and 55
seconds. `R` means red is inactive in shift 1; `B` means blue is inactive first.
At a boundary the next phase applies. Next-change countdowns skip boundaries
where that hub's activity stays unchanged; they include disable at match end.
Unavailable time or invalid game data does not silently select an alliance or
endgame. Disabled means scoring inactive. Unknown own alliance remains unknown.

These rules correct the previous Java-derived 125-second schedule in this fork.
Sources: [2026 game manual, sections 6.4–6.4.1](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm),
[WPILib 2026 game data](https://github.com/wpilibsuite/wpilib-docs/blob/main/source/docs/yearly-overview/2026-game-data.rst).
Real DS match time is approximate; simulated precision is not physical hub timing
calibration.

## WPILib audit

Pinned WPILib 2026.2.1 provides simulated enabled/autonomous/test/E-stop,
DS/FMS attached, alliance station, match time, game message, and event/match
metadata through DriverStationSim/HAL. The GUI offers manual state changes and
an enabled countdown, but no practice sequencer; its timer shortcuts seed 15/135
seconds, not 2026's 20/140. Practice adds orchestration and delayed game data;
season code supplies hub interpretation. This does not emulate FMS scoring,
refereeing, field networking, lighting, or six-robot field operation.
See [GUI FMS controls](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/glass/src/lib/native/cpp/other/FMS.cpp)
and [GUI DS/countdown](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp).

## Verification and current limits

Build with the local development Maven repositories described in the AutoDrive
guide. The latest full robot build covers desktop/roboRIO Debug and Release,
with 12 native tests in each desktop configuration. Focused tests cover official hub
boundaries, unknown state, full practice lifecycle, retained/repeated commands,
configuration snapshotting, interruption, reset, destruction, and restoration.

For an isolated GUI-loaded runtime check:

```sh
FRC_SIM_NT_PORT=5811 HALSIMWS_PORT=3301 FRC_SIM_GUI=1 ./scripts/sim_headless.sh
# In another terminal, using the documented simulator Python environment:
python scripts/practice_smoke.py --port 5811 --hal-port 3301 \
  --output artifacts/practice-match-delivery-gui-validation.json
python scripts/practice_smoke.py --controls-only --port 5811 --hal-port 3301 \
  --output artifacts/practice-controls-gui-validation.json
```

The full check observes all 163 seconds, game-data delivery, every hub phase,
shared context, autonomous cancellation, and a subsequent manual-disable event.
The controls check uses the same official profile, reaches TELEOP after 23
seconds, checks left-bumper ownership/release, verifies hub order and the
first shift, then aborts. `--auto-result R` and `--auto-result B` select both
FMS outcomes; `--station Blue1` or `--station Red1` selects the alliance. Tests target the
verified local process and never send DS mode updates during a practice run
except the deliberate abort. GUI-loaded evidence establishes extension
coexistence; the abort stimulus is a HAL state update, not an automated GUI click.

Linux Debug/Release clean consumers have linked and run the native controller
from the local vendordep. Windows validation/public publication remain pending.
The matching unpublished Base source ref must be supplied to the Simulation CI
workflow before cross-platform release checks can run.

Practice context/lifecycle checks do not establish navigation completion. The
separate current [headless AutoDrive smoke](../../artifacts/autodrive-onboard-headless-validation.json)
passes blue/red teleop goals and cancellation, but repeated autonomous navigation
still stops short. Its failure remains outside this context-only feature.

The earlier [practice validation record](../../artifacts/practice-mode-validation.json)
captures its binary/source hashes, GUI results, package checks, and limitations.
Current hub-order integration is reported separately with its own executable hash.

The 2026 hub-order integration was checked through the actual robot executable
with both [red](../../artifacts/hub-context-blue-red-auto-final.json) and
[blue](../../artifacts/hub-context-blue-blue-auto-final.json) AUTO-result messages.
For a Blue1 robot these exercise active and inactive shift 1 respectively. The
practice station mapping has explicit Blue1 and Red1 regressions: WPILib's zero
station value means unknown, so its alliance station values start at one.
