# AdvantageScope

The native robot uses WPILib NetworkTables struct topics and WPILOG files, both supported by AdvantageScope. This does not require AdvantageKit or Java replay files. The installed application used for validation is **AdvantageScope 26.0.0**.

## Live desktop simulation

Build and launch `scripts/sim_headless.sh`, then connect AdvantageScope to NT4 at `127.0.0.1` (port 5810). Keep the local Driver Station disabled while inspecting telemetry. This does not connect to or deploy to a physical robot.

Use the typed topics under `/AdvantageScope/Drive`:

| Topic under `/AdvantageScope/Drive` | Type | Visualization |
|---|---|---|
| `Pose` | `Pose2d` | 2D Field robot pose |
| `RobotPose` | `Pose3d` | 3D Field robot or coordinate axes; estimated planar pose lifted to z=0 |
| `Speeds` | `ChassisSpeeds` | Expand `vx`, `vy`, `omega` into line graphs; robot-relative metres/second and radians/second |
| `MeasuredStates`, `TargetStates` | `SwerveModuleState[]` | Swerve module states; FL, FR, BL, BR ordering |

These describe the Phoenix estimator and requested/measured states. A displayed robot pose is not an independent ground-truth sensor or a physical calibration result. Existing SmartDashboard arrays remain available for older tools. Typed snapshots publish at most 10 Hz with one shared capture timestamp; no NetworkTables or log serialization runs in the 250 Hz control callback. Its bounded snapshot copy includes the eight measured/target module states. No new controller or actuator path is introduced.

Mechanism graphs remain available at `/SmartDashboard/<name>/Position`, `Velocity`, `Voltage`, `Target`, and `Connected`. Rack position/rate is metres/metres per second; hood position/rate and flywheel rate use output rotations/rotations per second, as documented in the port contract. These scalar mechanism values are not radians. Gain arrays and disabled-only acceptance status remain under `/<name>/Gains`; AdvantageScope viewing does not change their tuning safety gates.

The season fork also publishes `Pose3d[]` fuel positions under `/AdvantageScope/Simulation`. They describe the native numerical simulation. They do not establish field-contact accuracy or real game-piece tracking.

## Recorded logs and reusable layout

Open a completed file from `logs/*.wpilog` through **File → Open Log**. WPILib DataLogManager records the NT values and their struct schemas. Expand the `NT:/AdvantageScope` tree. The versioned mechanism input records also live in WPILOG; their six-double replay schema is separate from the standard typed geometry topics used for visualization.

Validate a log and generate a layout using the installed application's actual parser:

```sh
node scripts/validate_advantagescope.cjs \
  --log logs/FRC_example.wpilog \
  --output artifacts/advantagescope-validation.json \
  --layout artifacts/advantagescope-layout.json
```

For the season fork, use its dimensions and speed limit:

```sh
node scripts/validate_advantagescope.cjs \
  --log logs/FRC_example.wpilog --require-fuel \
  --max-speed 5.24 --wheelbase .5588 --trackwidth .5588 \
  --output artifacts/advantagescope-validation.json \
  --layout artifacts/advantagescope-layout.json
```

Defaults match the base example: 4.572 m/s speed and 0.6731 m wheelbase/trackwidth. These settings scale the visualizer; they do not modify robot control or tuning. `--app` selects another `app.asar`; the validator deliberately rejects versions other than 26.0.0 until their worker/layout contract is reviewed. Node.js is required; no npm packages are installed.

Use **File → Import Layout** for the generated layout. AdvantageScope's import action replaces its current windows, so export your current layout first if you want to retain it. The generated layout contains a 2D pose, 3D pose axes, chassis-speed graphs (linear speed on the left axis and angular speed on the right), measured/target swerve states, and a Metadata tab. Optional fuel positions appear as coordinate axes without requiring a downloaded mesh. Select a robot/game-piece asset in the 3D controls for a rendered model; none is installed or changed by this tool. The portable Evergreen field avoids requiring a season field download.

`/Metadata` string topics appear in the Metadata tab; WPILOG shows them as `NT:/Metadata/...`. No AdvantageKit metadata namespace is required.

## What the validator establishes

The tool reads the installed ASAR archive, extracts its unmodified WPILOG worker, JavaScript indexer, and WASM indexer to a temporary directory, then invokes the same worker messages the application uses. It loads recorded struct schemas and checks that the four drivetrain types contain samples, decode into finite numeric child fields, and do not produce type warnings. It expects a healthy four-module fixture; nonfinite sensor fault samples can still be valid logs but fail this stricter fixture check. `--require-fuel` additionally requires a decoded simulation `Pose3d[]` topic; an empty array is valid. The JSON evidence includes the exact application version and worker hash.

This is parser evidence, not a GUI rendering test. Layout generation follows the installed 26.0.0 export/restore schema; graphical import, assets, GPU rendering, camera placement, and a real NT connection must be assessed separately. The tool does not edit AdvantageScope preferences, saved windows, application files, or robot logs. A missing schema or typed topic fails validation instead of treating a raw byte stream as successful visualization.
