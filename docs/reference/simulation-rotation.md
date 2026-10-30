> Historical detail: this record may describe earlier versions or experiments. Start with the [current guide](../simulation-rotation.md).

# Requested turn without chassis rotation

The running simulator accepted both signs of turn input: requested omega was
+/-3.111 rad/s and wheel-derived omega about +/-3.17 rad/s. Nevertheless, both
truth yaw and odometry yaw remained fixed near -0.9324 rad. Truth body height
was 0.120 m. Restarting into an upright state restored turns in both directions.
This was not missing joystick input or a scheduler delay.

## Mechanism and limit of the diagnosis

A focused standalone probe against the local MecoSimulation physics reproduces
this distinction with the same 53 kg chassis, motor ratios, current limits and
friction. It explicitly initializes the same model upright or upside down,
applies tangent module directions and motor voltage, and steps four seconds:

| Initial orientation | Settled body height | Body yaw rate | Average wheel speed | Total wheel normal force |
| --- | ---: | ---: | ---: | ---: |
| Upright | 0.2783 m | 5.879 rad/s | 2.798 m/s | 519.8 N |
| Upside down | 0.1200 m | approximately zero | 2.798 m/s | 0 N |

Source and output are `artifacts/overturned-physics-probe.{cpp,txt}`. The probe
links the local physics library, not the season executable. It demonstrates
that free-spinning unloaded wheels can imply turning in wheel kinematics while
the body stays still. It does not establish the exact event that overturned the
original running scene. Its original roll/pitch was not captured before restart;
overturning is the likely explanation, consistent with the observed height,
stationary body, and recovery. Full-rate circular, square and varying-input
experiments after restart did not reproduce a persistent overturned state.

The previous truth publisher fabricated zero roll and pitch, hiding this state
in 3D visualization. It now retains actual attitude from external-plant feedback
on the existing simulation owner thread. No public vendordep ABI changes or
extra physics steps are introduced. `/AdvantageScope/Drive/Pose` remains 2D.

The body heights above refer to the raw physics origin, now published at
`/Simulation/PhysicsBodyTruth`. `/Simulation/RobotTruth` subsequently gained a
floor-level model origin: upright ground is approximately Z=0, while an inverted
body still shows its rotated attitude and elevated model-floor anchor. The
rigid-body physics itself was not changed by that display-frame correction.
The later [hard-mounted contact correction](../rigid-wheel-contact.md) replaces the
spring model used for the historical height measurements above.

## Recovery and regression

In desktop **Teleop**, press **Back/View** to call the existing full scene reset:
upright body, origin pose, cleared wheel state and reset fuel simulation. This
binding is excluded from roboRIO builds. Start/Menu remains a heading reset and
cannot right an overturned body. The reset does not alter Driver Station enable.

The existing smoke check now requires truth heading to change during its turn
phase, in addition to wheel-derived speed tracking. It would reject the observed
zero-heading-change failure. With Back as the reset binding, the check passed:
truth rotated 6.693 rad in 2.5 seconds, with final wheel-derived omega 3.214 rad/s.
Evidence is `artifacts/spin-truth-smoke.json`. A separate Back-button reset
check returned truth and odometry from a translated/rotated state to the origin
with roll/pitch near zero (`artifacts/back-scene-reset.json`):

```sh
python scripts/sim_smoke.py --max-speed 5.24 --max-angular-rate 7 \
  --reset-button 7 --reset-enabled --small-input .15 \
  --output artifacts/spin-truth-repeat.json
```

Final Gradle build passed desktop and roboRIO Debug/Release targets; the existing
21 season tests passed in both desktop configurations. These are numerical
simulation observations, not physical robot commissioning or validated tipping
limits. No real control gains, center of mass, friction or tip behavior were
changed to force rotation.
