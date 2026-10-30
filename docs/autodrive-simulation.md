# Native AutoDrive and perception simulation

AutoDrive runs inside the C++ robot program on both roboRIO and desktop. Hold
pilot **left bumper** in enabled teleop, or choose **AutoDrive target** for
autonomous. The reusable command is supplied by MecoBase, using the pinned native
PathPlanner AD* backend and the deployed `pathplanner/navgrid.json`. No separate
Python script or external velocity-command stream is required.

The integrated candidate passed a [red strict match](../artifacts/autodrive-integration-strict-red-final.json)
and an [ownership and navigation smoke](../artifacts/autodrive-integration-smoke-final.json)
with the non-instrumented binary. A [blue strict match](../artifacts/autodrive-integration-strict-blue-pass-instrumented.json)
passed with temporary diagnostic telemetry enabled. These are desktop simulation
results; they do not establish hardware performance.

The earlier companion implementation and its validation records are historical
prototypes. Native C++ replaces that runtime; there is no planner-source copy in
this robot project. The additional Python SE(2)/terrain/training behaviors are not
claimed as native parity. The upstream [AutoDrive training repository](https://github.com/MECO-Robotics/AutoDrive/tree/d74b0a15e25ddbc6bacfbbb864674371629bb19e)
contains GPU-vectorized match training and experimental strategy policies; these
are not packaged as a C++ robot policy. Its pose-graph policy proposal describes
contracts still under development, and its diagnostic strategies do not by
themselves supply qualified executable goals. The desktop footprint route search
and terrain speed profile cover parts of the planning problem, not trained-policy
inference or training-system parity. The experimental offensive command below
adapts part of the upstream deterministic strategy. A deployment
would need an explicit observation/action contract, a qualified policy artifact,
and end-to-end validation with the target robot's localization and sensing.

## Build

This project uses MecoBase `2026.2.10` and MecoSimulation `2026.0.15`
from public Maven. MecoBase provides native planning and perception; the desktop
Simulation package supplies physics and synthetic camera rays.

```sh
JAVA_HOME=/home/brian/wpilib/2026/jdk ./gradlew build --max-workers=2
./scripts/sim_headless.sh
```

The same source builds the roboRIO executable. No physical deployment or motion
was performed during development.

An experimental desktop-only `AutoDrive/UseFootprintSim` toggle selects the
development footprint command. It uses the modeled .86 m square chassis and
axis-aligned field boxes intersecting its contact height; traversable pitched
ramps are excluded. This is simulation geometry, not a measured hardware
configuration. It requires MecoBase 2026.2.7 or a matching development package
and is not the default driver behavior. `scripts/footprint_smoke.py` exercises the toggle through
desktop HAL and NT on both alliances. The test can publish one perceived
rectangle as `[minX,minY,maxX,maxY]` in metres on `/Simulation/FootprintObstacle`;
this input is desktop-only and does not create a rigid physics body.
The [development run](../artifacts/footprint-sim-validation.json) also checked a
detour whose direct line intersects a modeled trench support. At 20 ms sampled
poses, the chassis remained at least 0.067 m clear of that support and finished
within 0.02 m of the target. This is numerical simulation evidence, not a
continuous physical clearance or hardware measurement.
A subsequent [rotation diagnostic](../artifacts/footprint-rotation-failure.json)
failed: reported arrival occurred near the heading tolerance, followed by drift
to 0.087 rad error. Estimated and simulated physical heading agreed at the final
sample. After reducing the simulated angular settling threshold and holding the
final waypoint longer, [two subsequent runs](../artifacts/footprint-final-settling.json)
ended within 0.06 rad and remained within that bound one second after arrival.
These settings are simulation-only and do not establish hardware calibration.
For a perceived obstacle injected ahead of the moving chassis, [two trials](../artifacts/footprint-dynamic-replan.json)
both stopped and resumed with positive sampled clearance, but only one reached
the goal. The other remained active near an intermediate waypoint. A separate
[earlier trial](../artifacts/footprint-dynamic-failure.json) injected the obstacle
overlapping the chassis and stopped without reaching the goal. Dynamic recovery
remains experimental and is not established by the passing trial.
After the follower stopped angular correction inside its tolerance, [two more
instrumented trials](../artifacts/footprint-zero-band-trials.json) both completed
the perceived-obstacle replan, but one static detour was still active at its
25-second cutoff. A [later run](../artifacts/footprint-adaptive-trial.json) with
a 45-second bound completed that detour in 25.6 seconds and the replan with
0.019 m minimum sampled separation from the virtual obstacle. These are limited
desktop trials; they do not establish worst-case timing or continuous clearance.
Two subsequent [non-instrumented runs](../artifacts/footprint-candidate-noninstrumented.json)
passed the same checks. Their static detours took 25.6 and 26.6 seconds, and
the minimum sampled virtual-obstacle separations were 0.060 and 0.058 m. The
robot was still moving when the stop was requested, so these results do not
establish a safe stopping distance or hardware readiness.
After adding a settling gate before each search, two more
[desktop runs](../artifacts/footprint-settle-validation.json) passed. In the
second run, approximately 0.5 seconds separated the sampled stop and resume
requests; measured translation was 0.013 m/s and rotation 0.039 rad/s at the
first resumed sample. The virtual-obstacle separations were about 0.061 m in
both runs. These are sampled results, not continuous clearance or physical
braking evidence; stale wheel and localization feedback could hide motion.
The separate desktop `AutoDrive/UseFootprintProfileSim` toggle enables the
experimental follower profile with flat terrain. The first [profile trial](../artifacts/footprint-profile-failure.json)
passed alliance arrivals, static detour, release and disable, but failed the
rotation and dynamic replan arrival checks: both commands remained active near
their targets. The test used a locally built MecoBase at `328c81a`; it does not
model ramp grade or establish hardware behavior.
A [diagnostic repeat](../artifacts/footprint-profile-trace.json) failed the same
two checks. Its final samples show heading repeatedly crossing the 0.02 rad
settling boundary; the commands never completed the required dwell.
With the MecoBase `78780a1` overshoot handling and a 0.01 rad correction
deadband, [two subsequent runs](../artifacts/footprint-profile-tighter.json)
passed all checks. Their detours took 22.4 seconds and their sampled virtual
obstacle separations were 0.0645 and 0.0637 m. Final rotation errors were 0.0530
and 0.0538 rad against a 0.06 rad test limit, leaving little margin. These
flat-terrain simulation results do not validate slopes or physical stopping.
The current profile experiment derives boundaries and directional grades from
the modeled field ramp plates. It assumes 4 m/s² acceleration and braking for
simulation, and zero rolling resistance; these are not robot measurements. A
direct modeled bump-crossing trial using MecoBase `81cc76d` did not arrive and
remained active near the first plate. This path is not ready for deployment.
The repeat also failed, with simulated pitch reaching 0.232 rad and height
0.070 m before stopping short. The terrain supplier describes the robot
centerline, not the contact of the complete chassis footprint with the plates.
A third [recorded run](../artifacts/footprint-ramp-summary.json) also failed:
the requested translation stayed above 0.97 m/s while body position barely
changed over its final 100 samples and wheel-derived speed remained nonzero.
This observation does not isolate the contact or drivetrain cause. The summary
links the compressed full traces.
An isolated [published-plant comparison](../artifacts/bump-plant-speed-probe.json)
used the same MecoSimulation `2026.0.14` archive and a simplified speed
controller, with the robot's configured ride height and foam damping. At a
1 m/s target it did not clear within 10 seconds, with 1,447 accumulated
backing contacts and 77 steps of bumper support while all wheels were unloaded. At
5.24 m/s it cleared, with 55 accumulated backing contacts and eight unloaded
samples. This supports high-centering as a modeled mechanism; it does not prove
the exact cause in the robot run or characterize physical hardware. The
comparison can be repeated with `scripts/bump_plant_speed_probe.py` and the
published desktop static adapter archive.
The opt-in simulation profile now uses a three-second pose-progress watchdog.
In [recorded stall tests](../artifacts/footprint-stall-watchdog-summary.json),
the initial 0.1 m/s request threshold missed a stall at 0.048 m/s. With an
illustrative 0.01 m/s threshold, two trials completed the staging checks and
stopped before the bump goal with zero request after 17.7 and 13.6 seconds,
including replanning attempts. This does not complete the crossing, prove a
physical stopping distance, or validate behavior with stale localization.
The subsequent MecoBase `fdf0c85` revision treats a stall as terminal rather
than retrying it. In [two simulated trials](../artifacts/footprint-terminal-stall-summary.json),
the staging checks passed and the ramp command stopped before its goal with
zero request after 8.4 and 7.3 seconds. These measurements include approach
and settling and do not establish physical stopping performance.

`AutoDrive/AvoidBumpsSim` is a separate, default-off desktop experiment that
maps modeled ramp plates to footprint-planner obstacles. Its value is sampled
at command start. In [recorded trials](../artifacts/footprint-avoid-ramp-summary.json),
two of three runs using the modeled .86 m footprint reached the alternate goal
with about .121 m minimum sampled bump separation; the other stopped in the
trench. Two trials with an extra 5 cm per-side planning margin both failed the
staging detour; that margin was removed. The earlier margin trial continued
from the wrong starting point after staging failed, so its later arrival is not
valid evidence for the intended route. The driver now stops when staging fails.
This option is not validated for reliable routing, hardware, or bump traversal.
The desktop command now publishes `/AutoDrive/FootprintOutcome` so an arrival,
stall, blocked attempt, no-route result, or other terminal outcome can be
distinguished. An [instrumented repeat](../artifacts/footprint-trench-outcome-summary.json)
of the earlier trench failure reported `no-route`. At the recorded post-stop
pose, a reduced planner regression showed the pose was clear but its containing
grid center was obstructed; the previous planner returned no route, while the
new nearby-center seeding finds a checked route. Two subsequent full desktop
trials with MecoBase `6bdbc56` reached the alternate goal, with approximately
0.121 and 0.123 m sampled bump separation. They do not prove that the earlier
live search started at the recorded post-stop pose or establish worst-case
reliability, continuous clearance, ramp traversal, or hardware behavior.
The desktop footprint experiment enables bounded target-change replanning for
`FollowFuel`, and a separate default-off `AutoDrive/TrackTargetSim` setting
exercises it with a dashboard target. In [two controlled trials](../artifacts/footprint-moving-target.json),
the target moved 0.65 m while the robot was driving; the command requested a
stop, resumed, and arrived at the new target. Wheel-derived translation speed
was about 0.79 m/s at the first zero request in both runs. This is not an
instantaneous physical stop, a camera/fuel tracking test, or hardware evidence.
The non-footprint `AutoDriveToPose` path still samples a fixed goal; the separate
`AutoDriveTrackTarget` command handles opt-in moving targets.
The footprint command now holds zero while a changed target stabilizes, with a
bounded wait before replanning. In two [synthetic-camera trials](../artifacts/footprint-camera-fuel-summary.json),
newly published fuel selections changed while active; the command stopped,
resumed, collected two modeled pieces, and stopped on trigger release. The
test follows the existing release-on-collection workflow. A separate test that
kept the trigger held collected fuel but exhausted the three-attempt target
change budget; continuous tracking is not established. An attempted association
heuristic also failed and was removed. These results do not validate a real
camera, identity tracking, or physical intake behavior.
Two further release-on-collection trials using `4bee96e` also passed: each
observed a freshly published target change, stopped, collected while active,
and stopped on release. One resumed after the target-change stop; in the other,
collection occurred before resumption. The tests do not require a needless
restart after collection and do not establish continuous following.
After the profile envelope was made to override request slew limiting, a
[further run](../artifacts/footprint-profile-envelope.json) passed the same
checks using MecoBase `26cdf21`. It took 21.4 seconds for the detour, ended the
rotation 0.0462 rad from target, and sampled 0.0612 m minimum virtual-obstacle
separation. This is one run and does not establish continuous clearance.

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
heading and speed, and no driver retries. The onboard command can make up to
three obstacle-aware pathfinding attempts when a timed trajectory ends short;
its final straight approach still requires a clear navgrid segment. The scenario
stages autonomous before the neutral fuel grid and uses a centered clear lane so
teleop can collect the fuel without first pushing it away.
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

### Native moving-target experiment

The development MecoBase tracking command is selected for `FollowFuel`, or for
`TrackTargetSim` on desktop. It retains the ordinary fixed-goal command when
those options are off. One desktop run changed the dashboard goal by 0.65 m;
the command requested a stop, resumed and finished 0.028 m from the new goal
([trace](../artifacts/native-moving-target-trial.json)). At the first zero
request the simulated measured translational speed was approximately 0.61 m/s;
this is not an instantaneous or physical stopping guarantee.

A separate synthetic camera run detected a moving selection, stopped and
resumed, collected three simulated pieces, and stopped when the driver released
the command ([trace](../artifacts/native-camera-fuel-trial.json.gz)). The
linked executable used local MecoBase development archive SHA-256
`53f400d904c74b55b5e4dbfce0926587e826e1909646a806f51ebc1b608d9fca`;
its Gradle link input matched that archive. These are single desktop trials,
not evidence for real camera calibration or physical stopping distances.

The subsequent MecoBase `1918b55` tracking gate also waits for pose-derived
and wheel-derived motion to settle before replanning; it ends the command if
that wait times out. In the [first run](../artifacts/native-moving-target-settled.json),
the shifted target finished 0.0052 m away. In a [repeat](../artifacts/native-moving-target-settled-repeat.json),
the moving-target check finished 0.031 m away and resumed at approximately
0.046 m/s wheel-derived translation, while the preceding blue fixed-goal check
stopped 0.198 m short. The full repeat therefore failed; the fixed-goal stop
is unresolved. A separate [synthetic-camera run](../artifacts/native-camera-fuel-settled.json.gz)
passed its collection and release checks. These runs used the development
archive SHA-256 `d53be1b21ad5f573d31c66b36071c12f351e3e9e1a43e6ece9a808d7f788c099`,
matching the linked Gradle input. The gate cannot detect movement absent from
both pose and wheel feedback and does not establish physical stopping distance.
Two further instrumented runs passed both fixed goals and the moved-goal check
([first](../artifacts/native-moving-target-settled-instrumented.json.gz),
[second](../artifacts/native-moving-target-settled-instrumented-repeat.json.gz)).
Their per-tick estimated pose, simulated truth, requested and measured speeds,
and command status can help investigate a future recurrence; they do not explain
the earlier short stop.
A later [eight-run fixed-goal series](../artifacts/native-fixed-goal-diagnosis.json.gz)
and [six-run series following a moving-target run](../artifacts/native-fixed-after-move.json.gz)
all arrived. The [preceding moving-target run](../artifacts/native-fixed-sequence-setup.json.gz)
is retained to identify the simulator sequence. These bounded repetitions did
not reproduce or explain the earlier short stop; they do not establish a failure
rate or validate physical behavior.

The robot also completed a full local desktop and roboRIO Debug/Release build
against the development package with Base source `5fd8ffe`; desktop tests passed
24 cases in each configuration. The [build record](../artifacts/autodrive-development-consumer-build.json)
records matching linked-library and package hashes for all four targets. This is
a local development package check, not public Maven, Windows, or hardware evidence.

The [staged-package record](../artifacts/autodrive-staged-consumer-validation.json)
adds clean desktop and roboRIO Release consumer links against the CI-built
`2026.2.7` candidate, including the new reusable APIs. The candidate source
predates the final test-only changes; this is not a public-package check.
The full robot build was subsequently repeated with the staged `2026.2.7`
coordinate: desktop and roboRIO Debug/Release linked successfully, with 24
passing tests in each desktop configuration. The [staged robot build record](../artifacts/autodrive-staged-robot-build.json)
checks the linked archive hashes. This does not establish public Maven availability.
A [strict blue practice-match trial](../artifacts/autodrive-staged-strict-blue.json)
using the staged `2026.2.7` binary passed all checks, including 20 goal legs,
no retries, bounded requests, simulated fuel following and collection, a
simulated scoring volley, and stop on completion. This is one desktop run; it
is not evidence of real-camera calibration, physical scoring, or hardware safety.
A clean robot build without local overrides also passed against public MecoBase
`2026.2.7`: desktop and roboRIO Debug/Release linked, and both desktop test
configurations passed 24 cases. The [public build record](../artifacts/autodrive-public-consumer-build.json)
compares each linked library with its checksummed public archive and verifies
that the previous `2026.2.6` POM remains unchanged.
A [strict red practice-match trial](../artifacts/autodrive-public-strict-red.json)
using the public MecoBase `2026.2.7` binary passed all checks with 22 goal legs,
no retries, bounded requests, simulated fuel following and collection, a scoring
volley, and stop on completion. This is desktop simulation evidence, not a
physical-camera, drivetrain, or scoring qualification.

### Explicit robot footprint configuration

`AutoDrive/UseConfiguredFootprint` is a separate, default-off mode available on
both desktop and roboRIO. At robot startup it reads `autodrive-footprint.json`
from the deploy directory. The file must supply a version, measured robot length
and width, a nonnegative per-side margin, field bounds, at least one obstacle
rectangle, planner resolution and expansion budget, and motion limits. Distances
are metres, speed is m/s, acceleration m/s², angular speed rad/s, and angular
acceleration rad/s². The selected speed and angular speed must not exceed the
drive's configured limits. Missing, malformed, invalid, or over-limit input
causes the selected command to stop; `AutoDrive/ConfiguredFootprintStatus`
reports the load or rejection, and `/AutoDrive/FootprintOutcome` reports its
terminal result. Restart the robot after changing the file.

The shape below illustrates the schema, not a complete field map or validated
robot configuration. Replace every value with robot-specific and field-specific
measurements before use. Each obstacle is an axis-aligned field-frame rectangle;
the configured mode has only this static obstacle snapshot and does not detect
other robots or obstacles absent from the file. Without a `terrain` section it
uses stop-at-waypoint following. Do not infer hardware readiness from the example.

```json
{
  "version": 1,
  "footprint": {"length_m": 0.86, "width_m": 0.86, "margin_m": 0.05},
  "field": {"min_x": 0, "min_y": 0, "max_x": 16.541, "max_y": 8.069},
  "obstacles": [{"min_x": 7.9, "min_y": 3, "max_x": 8.6, "max_y": 5}],
  "planner": {"resolution_m": 0.2, "expansion_budget": 30000},
  "limits": {"max_speed_mps": 1, "max_omega_radps": 1,
             "max_acceleration_mps2": 2, "max_alpha_radps2": 3}
}
```

Desktop HAL/NT tests using an illustrative configuration checked
[blue and red fixed goals](../artifacts/configured-footprint-valid.json), a
[moving goal](../artifacts/configured-footprint-moving.json), and zero requests
with [missing](../artifacts/configured-footprint-missing.json.gz) and
[over-limit](../artifacts/configured-footprint-over-limit.json.gz) configurations,
and [release while moving](../artifacts/configured-footprint-release.json.gz).
In the moving test the first zero request occurred at approximately 1 m/s
wheel-derived speed, and the first resumed request at approximately 0.055 m/s.
These are simulation samples, not measured stopping distances or physical
validation. The fixture used no margin and one illustrative obstacle, and is
not deployed or suitable as a competition field map.

An optional `terrain` object enables the MecoBase terrain profile on both
desktop and roboRIO. It requires `lateral_acceleration_mps2`,
`steering_rate_radps`, `stall_timeout_s`, `stall_progress_m`,
`stall_request_mps`, and a nonempty `regions` array. These watchdog thresholds
must be positive, and the request threshold must be below the speed limit. Each region supplies a
`bounds` rectangle, `grade_x` and `grade_y` as rise/run in the positive field
axes, and dimensionless `rolling_resistance`. Regions must lie inside the field
and must not overlap; ambiguous travel along a shared boundary fails closed.
Supply robot-specific measured values. The profile models the path centerline
and uses the configured acceleration for braking; the watchdog ends a stalled
route when pose progress remains below the supplied threshold while the request
exceeds its threshold. It cannot detect motion missing from the pose estimate,
model whole-body contact, guarantee traction, or establish a physical stopping distance.

The [simulation-only watchdog fixture](../artifacts/configured-terrain-watchdog-fixture.json)
shows the current optional schema. It is deliberately not installed in the deploy
directory and is not a competition field map. Two earlier desktop HAL/NT runs with
the [pre-watchdog fixture](../artifacts/configured-terrain-simulation-fixture.json) and
virtual terrain over a flat physical lane passed blue/red goals and release
stopping ([first](../artifacts/configured-terrain-simulation.json.gz),
[repeat](../artifacts/configured-terrain-simulation-repeat.json.gz)); the repeat
recorded 57 active samples in each virtual region. They do not validate ramp
traversal. A [third run with unsplit-boundary rejection](../artifacts/configured-terrain-boundary-guard.json.gz)
passed the same checks using MecoBase `2026.2.8-dev.2`. The published
`2026.2.8` release provides the same terrain API without an override.

Two additional configured-profile trials used a [simulation-only bump fixture](../artifacts/configured-terrain-bump-fixture.json)
derived from the published MecoSimulation `v2026.0.14` field-collider source
(commit `f6d0db7ce343c28a1d7ab59378c12738e1406e1d`; extracted source SHA-256
`01917a3a6bc48b9dd41c10bbb30f31d2253f4f63c2c1b9d9974f087dc0f8dd08`).
The fixture uses the modeled centerline grades and illustrative 4 m/s²
acceleration and watchdog settings; it is not a measured robot configuration.
Both [first](../artifacts/configured-terrain-bump-stall.json.gz) and
[repeat](../artifacts/configured-terrain-bump-stall-repeat.json.gz) reached the
staging goal, then ended with `stalled` and zero requested velocity before the
bump goal, after approximately 7.3 and 8.4 seconds respectively. Neither
crossed the bump. This establishes the configured watchdog path in desktop
simulation, not the physical cause, braking distance, or hardware performance.

The [staged consumer record](../artifacts/configured-terrain-staged-robot-build.json)
checks the full robot build against the CI-built `2026.2.8` candidate for desktop
and roboRIO Debug/Release. Both desktop test configurations passed 25 tests,
and each linked library matched its staged archive. This is not public Maven or
hardware validation. The subsequent [public consumer record](../artifacts/configured-terrain-public-robot-build.json)
confirms the clean robot build and linked library hashes against published Maven.
A separate [missing-watchdog-field test](../artifacts/configured-terrain-missing-watchdog.json.gz)
used an intentionally [invalid fixture](../artifacts/configured-terrain-missing-watchdog-fixture.json);
selecting the mode rejected the configuration and kept requested motion at zero.
A separate [simulation-only avoidance fixture](../artifacts/configured-terrain-bump-avoidance-fixture.json)
adds the modeled bump-plate plan-view rectangles as obstacles. In [two trials](../artifacts/configured-terrain-bump-avoidance.json.gz)
([repeat](../artifacts/configured-terrain-bump-avoidance-repeat.json.gz)), the
configured command reached the goal on the other side without traversing those
rectangles; minimum sampled separations from the modeled bump were about 0.121
and 0.123 m. Both used the staged `2026.2.8` candidate and illustrative robot
limits. The field boxes, clearance samples, and modeled robot are not physical
measurements, and this does not establish ramp traversal or continuous clearance.
An [overlapping-region configuration](../artifacts/configured-terrain-overlap-fixture.json)
was also [rejected through the scheduled command](../artifacts/configured-terrain-overlap.json.gz),
with zero requested motion throughout the desktop check.

## Experimental offensive command

`AutoDrive offensive` is an autonomous chooser entry. In teleop, set
`AutoDrive/OffenseEnabled` and hold the pilot left bumper; releasing it cancels
and stops the drive and mechanisms. The ordinary target-drive binding remains
available with the toggle off. `AutoDrive/OffensePhase` reports the current
stage. Unknown alliance or HUB state inhibits decisions and cancels collection;
HUB shooting requires an active HUB and a settled return to the selected shot
pose. Ferry shots use the existing ferry trajectory safety and readiness checks.

This is an adaptation of the deterministic offense in upstream AutoDrive
[`_tensor_3v3_actions.py` at d74b0a15](https://github.com/MECO-Robotics/AutoDrive/blob/d74b0a15e25ddbc6bacfbbb864674371629bb19e/frc_defense/_tensor_3v3_actions.py).
The active-HUB pickup ranking uses its acceleration-limited pickup and return
time per confidence-weighted nearby yield; inactive-HUB pickup excludes the friendly alliance zone and uses a
nearby target before a ferry attempt. When no fuel is visible, the command
searches toward the neutral zone. It briefly holds a pickup target through
observation dropouts. Finished or failed collection navigation is retried
after 0.3 seconds while the collection attempt remains active. The command
retains a score or ferry commitment across failed navigation and incomplete
feeding, retrying after a 0.5-second pause. A full feed attempt or a HUB-state
change releases it. Return from outside the alliance zone approaches a
fixed neutral-side trench waypoint before the shot position; it retries the
waypoint at most three times at a 1 m/s path-output limit and requires settled
arrival before the final leg. This route is intended for the modeled field and has not been
physically validated.

This robot has no carried-FUEL sensor. The command therefore runs bounded
collection and shooting attempts and cannot reproduce upstream capacity or
possession decisions. It estimates pickups from observations disappearing near
the intake; occlusion and missed detections can make this count wrong. A
completed feeding attempt clears the estimate without proving that the hopper
is empty. Collection reevaluates the active-HUB nearby-fuel and inactive-HUB eligibility
rules continuously. Autonomous starts returning at 13 seconds remaining to leave time for a
shot, including when possession is unknown. Timed attempts can therefore
run with an empty hopper. A 20-second collection limit
prevents indefinite collection. The optional
`AutoDrive/EstimatedCapacity` preference defaults to zero (unknown); a positive
value ends collection when the uncertain estimate reaches that threshold. Its
camera adapter provides confidence and capture time but lacks persistent track
identities, velocity, teammate intent and opponent tracks. The candidate
ranking omits obstacle-detour costs,
and the upstream cluster-edge sweep; it is not policy parity. Simulation
camera observations feed the same perception interface as hardware; the policy
does not read simulated stored-fuel truth. Desktop scoring results do not prove
physical sensing, mechanism calibration or field safety.

The [public-package blue practice run](../artifacts/offensive-public-blue.json.gz)
scored 7 FUEL in autonomous and 35 more in teleop, with 92 teleop acquisitions;
54 remained stored at completion. Staged-package runs also scored in
[red](../artifacts/offensive-staged-red-repeat.json.gz), and a
[cancellation probe](../artifacts/offensive-staged-safety.json) checked
drive requests and indexer/conveyor outputs. These runs use simulated cameras
and physics. An earlier red run crashed with `std::bad_optional_access` near
match end; two later red runs passed, but the original crash was not explained.
One blue run spent several seconds on a modeled bump during a later return.
These results do not establish reliable navigation or hardware readiness.
See the [evidence record](../artifacts/offensive-simulation-evidence.md) for
package hashes, test conditions, and limitations.
