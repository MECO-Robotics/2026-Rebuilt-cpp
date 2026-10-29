# Native AutoDrive and perception simulation

AutoDrive runs inside the C++ robot program on both roboRIO and desktop. Hold
pilot **left bumper** in enabled teleop, or choose **AutoDrive target** for
autonomous. The reusable command is supplied by MecoBase, using the pinned native
PathPlanner AD* backend and the deployed `pathplanner/navgrid.json`. No separate
Python script or external velocity-command stream is required.

The earlier companion implementation and its validation records are historical
prototypes. Native C++ replaces that runtime; there is no planner-source copy in
this robot project. The additional Python SE(2)/terrain/training behaviors are not
claimed as native parity.

## Build

This worktree consumes unpublished development vendordeps:

- MecoBase `2026.2.5-dev.1`: native command, validation, ownership and native AD* integration.
- MecoSimulation `2026.0.14-dev.1`: physics and synthetic camera rays only, desktop-only.

```sh
JAVA_HOME=/home/brian/wpilib/2026/jdk ./gradlew build --max-workers=2 \
  -PmecoBaseMaven=../MecoBase-cpp-integration/build/vendor-maven/maven \
  -PmecoBaseVersion=2026.2.5-dev.1 \
  -PmecoSimulationMaven=../MecoSimulation-cpp-integration/build/integration-package/maven \
  -PmecoSimulationVersion=2026.0.14-dev.1
./scripts/sim_headless.sh
```

The same source builds the roboRIO executable. `./gradlew deploy` would require
the same local-Maven properties until these versions are publicly released. No
physical deployment or motion was performed during development.

## Controls and real detections

SmartDashboard `AutoDrive/TargetX`, `TargetY` and `TargetHeading` select an absolute
blue-origin field target in metres/radians, on either alliance. The target is
sampled on command start; release and press LB again to choose another goal.
Start's driver-heading reset does not change this field frame. Ensure actual field
localization is initialized and the navgrid fits the real footprint before use.

`AutoDrive/FollowFuel` selects the nearest fresh cluster from front and rear
camera detections. Hardware reads Jetson detections from
`/Perception/front/detections` and `/Perception/rear/detections` through MecoBase's
object-detection IO abstraction. The Jetson detector service must publish those
frames before hardware FUEL following can operate.

Both camera mounts default to **15° downward** (WPILib pitch +15°), with front
yaw 0° and rear yaw 180°. Configure mounts through Preferences under
`Perception/Front/` and `Perception/Rear/`: `X`, `Y`, `Z` in metres and
`RollDeg`, `PitchDeg`, `YawDeg` in degrees. Defaults are X ±.43 m, Y 0 m,
Z .54 m. Existing saved Preferences override these defaults. These nominal
positions still require physical measurement. The front transform follows the
hopper as intake extension changes; the rear transform stays fixed.

Desktop simulation injects typed camera frames into the same perception path.
The MecoSimulation sensors use physical pose to generate visibility, occlusion,
noise and dropout; projection and clustering use estimated robot pose and
capture-time mount transforms. Sensor range is 6 m, horizontal/vertical FOV
120°/90°, dropout 2%, angular noise .002 rad, and seeds 3737/3738. Only unheld
floor FUEL is sensed. The FUEL cluster radius is .15 m. Observations expire
after .15 s, including when a camera reports empty frames. Intake operation remains
the existing right-trigger binding.

`/AutoDrive/Active` and `/AutoDrive/Status` report command state.
`AutoDrive/DetectionFresh` and `AutoDrive/ClusterCount` are SmartDashboard
diagnostics. There is no `/AutoDrive/Command` input topic.
`AutoDrive/SelectedFuelX` and `SelectedFuelY` show the selected cluster when
following FUEL. A short approach stops if its segment is blocked or the navgrid
is unavailable; `stopped-before-goal` distinguishes that result from arrival.

## Validation

```sh
python scripts/autodrive_smoke.py --output artifacts/autodrive-onboard-validation.json
```

This test uses Python only as a HAL/NT test driver; it starts no companion or
planner. It checks navigation on both alliances, autonomous execution, release
and disable ownership, typed synthetic detection and absence of an external
command stream. `--port` and `--hal-port` support isolated simulator endpoints.
The native Base test protects missing-target stopping and ownership; its existing
object-perception regressions cover capture-time moving mounts, clustering,
NT frame validation and expiry. MecoSimulation's artifact
consumer covers the synthetic sensor's visibility and occlusion. Build/link and
numerical simulation evidence do not establish real detector calibration or
hardware commissioning.

`scripts/autodrive_full_match.py` drives a practice-match scenario through the
robot's existing controls. Its `--shoot-at-home --follow-fuel` options exercise
shooting and a camera-selected pursuit. The script selects goals and operates
controls as a simulated driver; it is not an onboard match strategy. Simulated
hub scoring uses the match context and an approximate three-second assessment
grace after deactivation, not the field's physical scoring sensors.

Add `--continuous` to `--shoot-at-home --follow-fuel` to exercise repeated
collection and scoring through the official practice match, including commanded
wheel motion in the last ten seconds and a stopped robot at completion. The
driver waits for a suitable active hub window, records each volley after the
projectiles settle, and permits up to two separately recorded bumper retries
for a valid goal stopped less than 1 m away. A retry requires releasing and
pressing the binding again; it is not autonomous recovery by the robot.
Add `--strict-arrival` to require bounded AutoDrive outputs, settled position,
heading and speed, and no driver retries.
For example, with the isolated simulator running on ports 5811 and 3301:

```sh
python scripts/autodrive_full_match.py --alliance blue --auto-result R \
  --shoot-at-home --follow-fuel --continuous --output artifacts/sustained-blue.json
```

One sustained red attempt stopped 0.77 m from a return goal; another blue
attempt reported arrival but settled 0.38 m from its target. Their underlying
causes are unresolved. Passing runs do not establish that every approach succeeds.

The recorded headless matches cover [blue with result R](../artifacts/goal-full-match-blue-final-attempt.json),
[red with result B](../artifacts/goal-full-match-red-feed-diag.json), and a
[repeated red match with result R](../artifacts/goal-full-match-red-repeat-window.json).
They check hub context, navigation, intake, a simulated score, camera pursuit,
and the disabled stop. The simulated indexer can take several seconds to reverse
after sustained intake; the scenario allows a longer active-hub shooting window.
These runs do not validate physical hub timing, camera calibration, or hardware.

### FMS hub context

On both hardware and simulation, AutoDrive receives the same `MatchContext`
computed from Driver Station match time, alliance and game-specific message.
The message names the alliance **inactive first**; the other alliance is active
in shift 1. AutoDrive can read both alliance-order fields, own hub activity and
`ownNextChange` through its read-only `MatchState()` supplier. Unknown game data,
alliance or timing remains unknown; dashboard known flags accompany sentinel
values. The fields are also published under `/SmartDashboard/AutoDrive/` as
`HubOrderKnown`, `InactiveFirstAlliance`, `FirstActiveAlliance`,
`HubNextChangeKnown`, and `HubNextChangeSeconds`.

This supplies official timing context; it does not choose a goal or shooting
strategy based on hub state. Driver Station time is approximate on a real field.

See the [hub-context validation record](../artifacts/hub-context-validation.json)
for build, packaged-consumer and simulated FMS evidence.

### Simulation reset and autonomous start

Desktop autonomous startup waits until the five-loop drivetrain sensor/pose
reconciliation after a world reset completes before scheduling the selected
command. This prevents PathPlanner from sampling a pose or speed left over from
the previous run. The physical-robot autonomous start remains immediate.
The existing `autodrive_smoke.py` covers the repeated-goal regression, including
a full reset after prior driving, with blue and red teleop checks.
