# Where to make changes

Start with the task you want the robot to do. These files contain this season's robot-specific choices; the shared drive, mechanism, and motor IO implementations come from the vendordeps.

For a first change, find the closest existing setting or command in the table, follow its uses in the editor, and change one thing at a time. A header (`.h`) usually declares an interface or holds settings; its corresponding source file (`.cpp`) implements behavior. The `season` folders contain this robot's choices. You normally do not need to edit the shared vendordeps for a season-specific change.

| I want to… | Edit here |
| --- | --- |
| Change an autonomous route or sequence | Open [PathPlanner](../src/main/deploy/pathplanner/) and edit its `paths` and `autos` in the PathPlanner app. |
| Add an autonomous action or chooser option | Edit [RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp). Named commands can be used as PathPlanner event markers. |
| Change the spin-up timeout or Choreo chooser gains | Edit the settings at the top of [RobotAutos.h](../src/main/include/season/auto/RobotAutos.h). |
| Change a controller button | Edit `ConfigureBindings()` in [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp). |
| Change what an intake, conveyor, indexer, hood, or shooter command does | Edit [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). The command interfaces are in [Mechanisms.h](../src/main/include/season/mechanisms/Mechanisms.h). |
| Change mechanism CAN IDs, ratios, gains, limits, or controller CAN rates | Edit [MechanismConfig.h](../src/main/include/season/mechanisms/MechanismConfig.h). |
| Change mechanism voltages, positions, or shooter presets | Edit [MechanismBehaviorConfig.h](../src/main/include/season/mechanisms/MechanismBehaviorConfig.h). |
| Change drivetrain hardware or swerve odometry rate | Edit [DriveConfig.h](../src/main/include/season/drive/DriveConfig.h). |
| Change driver deadband or aiming gains | Edit [DriveBehaviorConfig.h](../src/main/include/season/drive/DriveBehaviorConfig.h). |
| Change QuestNav or AprilTag camera mounts or vision freshness | Edit [VisionConfig.h](../src/main/include/season/vision/VisionConfig.h). |
| Change object-perception camera mounts, detection range, clustering, or update period | Edit [PerceptionConfig.h](../src/main/include/season/vision/PerceptionConfig.h). |
| Change unloaded pit-check speeds, tolerances, or timeouts | Edit [SystemCheckConfig.h](../src/main/include/season/diagnostics/SystemCheckConfig.h). |
| Change the standard robot loop or controller ports | Edit [RobotControlConfig.h](../src/main/include/season/RobotControlConfig.h). |
| Change the shot table or shooting limits | Edit [Shooting.h](../src/main/include/season/shooting/Shooting.h). Shared movement limits apply to both the moving-ferry solver and shot readiness. Shot calculations are in [Shooting.cpp](../src/main/cpp/season/shooting/Shooting.cpp). |
| Change AutoDrive or offensive planning settings | Start with the constants at the top of [AutoDrive.h](../src/main/include/season/drive/AutoDrive.h) or [OffensiveAutoDrive.h](../src/main/include/season/drive/OffensiveAutoDrive.h). |
| Change the bumper-inclusive robot length or width | Edit [RobotGeometry.h](../src/main/include/season/RobotGeometry.h). Navigation, scoring clearance, and simulated placement use these dimensions; simulation contact details have separate estimates. |
| Change a simulation assumption | See [simulation configuration](#simulation) below. |

## Autonomous

PathPlanner routes and auto sequences live under [deploy/pathplanner](../src/main/deploy/pathplanner/). Edit them in PathPlanner so the app maintains their file format. The robot loads PathPlanner autos into the `Auto Choices` chooser.

[RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp) registers event markers and adds the C++ chooser options. Use the registered name exactly in PathPlanner; existing autos use the legacy names below. Commands running together cannot own the same subsystem. In particular, `AimedHubShot` and `AutoAim` own the drivetrain, so do not run either alongside a path that owns it.

| Marker | Action, requirements, and end behavior |
| --- | --- |
| `DeployIntake` / `StowIntake` | Set the rack target once; own the rack during the instant command. Stow targets 0 m. |
| `SpinIntake` | Runs intake and conveyor until interrupted; owns and stops both. |
| `Agitate` | Runs the two indexers until interrupted; owns and stops both. |
| `FeedRollers` / `CoordinatedHubShot` | Aliases for a bounded coordinated hub shot; own hood, shooter, conveyor, and both indexers. |
| `AimedHubShot` | Coordinated hub shot that also owns and commands the drivetrain; ends on feed completion or timeout. |
| `AutoAim` | Aims and spins the shooter until interrupted; owns and stops drivetrain, hood, and shooter. |
| `AutoSpinUp` | Calculates hood and shooter targets for up to two seconds; owns and stops both. |
| `Fender` | Sets a hub preset once; briefly owns hood and shooter and leaves targets active. |
| `IdleRollers` | Stops conveyor and indexers once; briefly owns those mechanisms. |

The physical-robot chooser defaults to **Hub shot (vision required)**, an active shooting action; desktop simulation defaults to **AutoDrive offensive**. Select **Do nothing** for no autonomous action. The [AutoDrive guide](autodrive-simulation.md) explains the other choices and shot configuration.

## Example: follow the intake button

1. Find the right-trigger binding in `ConfigureBindings()` in [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp). It schedules `Acquire` while the trigger is held during teleop.
2. Follow `Acquire` to [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). Its first `RunEnd` function deploys the rack and runs the intake and transport.
3. The last argument lists the subsystems the command owns. Another command requiring one of them can interrupt it. The second function stops the acquisition mechanisms when the command ends, including when the trigger is released.
4. After a change, build and run the simulator. Enable teleop, hold and release the trigger, and check both the action and its cleanup. Simulation is not proof of physical direction or calibration.

In a binding, `WhileTrue` schedules the command when the condition becomes true and cancels it when the condition becomes false; it does not repeatedly restart a command that finishes while still held. `OnTrue` schedules on the rising edge, so the scheduled command may keep running after the button is released. A command's requirements give it exclusive control of its subsystems: scheduling another command that needs one can interrupt the first. Check the command's end function as well as its start and run functions before changing a binding.

The pilot POV-down action named `Stow` retracts only to the clearance-safe position while held, whereas the `StowIntake` autonomous marker targets the fully stowed position. Check the intended position before changing either.

## Mechanisms and constants

Start with the existing commands in [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). Commands that run motors while held need a stop or cleanup path when they end or are interrupted. For a new mechanism, use the shared MecoBase mechanism and motor IO interfaces rather than accessing a vendor controller directly in a command. [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp) constructs the season subsystems and binds commands to controls.

Put robot-specific settings in the configuration header for the subsystem or behavior they configure. Use `inline constexpr` for values that can be computed at compile time; configuration objects and calculations that need runtime evaluation use `inline const` or a function. Keep units and coordinate systems clear: rack travel is in metres, hood position is in output rotations, and flywheel speed is in rotations per second. Check current limits, motor direction, and travel limits against the actual robot before applying changes.

Some values are supplied by WPILib Preferences or the dashboard at runtime. Changing a header default does not necessarily replace an already-saved preference. See the [AutoDrive guide](autodrive-simulation.md) for the shooting and feed preferences and the [control timing guide](control-timing.md) for the standard loop and requested CAN, sensor, and input rates.

Object-camera mounts use saved `Perception/Front/` and `Perception/Rear/` Preferences (`X`, `Y`, `Z`, `RollDeg`, `PitchDeg`, and `YawDeg`); existing values override source defaults. The front camera also moves with the intake extension. QuestNav and AprilTag camera mounts in `VisionConfig.h` are configured directly. For a swerve CAN ID, check both the named module fields and the CAN bus in `DriveConfig.h`.

### Example: update a measured shot

The hub calibration arrays in [Shooting.h](../src/main/include/season/shooting/Shooting.h) are paired by index: each entry in `kDistanceInches` corresponds to the hood output rotations in `kHoodTable` and flywheel rotations per second in `kFlywheelTable` at the same position. Keep the distances in increasing order and update the measured row together. There are currently eight entries; changing the number also requires updating the interpolation function in [Shooting.cpp](../src/main/cpp/season/shooting/Shooting.cpp). Check the calibrated distance bounds when changing the endpoints. The table's flywheel values are wheel speeds, not ball speeds. Presets for fixed driver actions are separate from this calculated shot table; trace the binding and command to identify which one you are changing.

Record the measurement conditions and units with the team's calibration notes. Build, exercise the relevant shot in simulation, and have the team verify the physical shot and limits before relying on it in a match.

The simulated HUB shot uses a ballistic speed rather than the physical shot table; see [simulation limits](autodrive-simulation.md#limits-and-troubleshooting). The driver HUB preset can also use `TunableNumbers/ShooterCalibration` values when calibration is enabled at command start; check that mode when a preset does not match its source default.

## Simulation

- [SimulationConfig.h](../src/main/include/season/simulation/SimulationConfig.h): fuel behavior, motor model, contact estimates, simulation timing, sensors, battery, and practice settings.
- [FuelSimulation.cpp](../src/main/cpp/season/simulation/FuelSimulation.cpp): robot-state inputs, transport motor loads, and simulation telemetry.
- [FuelModel.cpp](../src/main/cpp/season/simulation/FuelModel.cpp): fuel particle motion, collection, scoring, and model state transitions.
- [Simulation.h](../src/main/include/season/simulation/Simulation.h): public desktop simulation interfaces, including the fuel and loaded-motor model contracts.

Simulation settings are estimates; they do not establish physical calibration. See [simulation field](simulation-field.md) for the model and its limits.

## Check your change

Run `./gradlew build --max-workers=2` from the repository root, then use `./gradlew simulateNative` and exercise the affected controls or autonomous choice. For a held action, also release it and try another action that uses the same subsystem; observe what stops and what remains commanded. For an autonomous change, select the actual chooser option and check its start, finish, and interruption behavior. A successful build checks compilation and the existing tests; a simulation check shows only simulated behavior. Before deploying or moving the physical robot, review the change with the team and verify the relevant hardware settings and safe operating conditions.
