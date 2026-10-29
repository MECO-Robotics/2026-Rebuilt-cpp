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
inference, strategic decision making, or training-system parity. A deployment
would need an explicit observation/action contract, a qualified policy artifact,
and end-to-end validation with the target robot's localization and sensing.

## Build

The project resolves MecoBase `2026.2.6` and MecoSimulation `2026.0.14`
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
configuration. It requires a local development MecoBase package and is not the
default driver behavior. `scripts/footprint_smoke.py` exercises the toggle through
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
