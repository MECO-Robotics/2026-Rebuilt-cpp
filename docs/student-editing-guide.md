# Where to make changes

Start with the task you want the robot to do. These files contain this season's robot-specific choices; the shared drive, mechanism, and motor IO implementations come from the vendordeps.

| I want to… | Edit here |
| --- | --- |
| Change an autonomous route or sequence | Open [PathPlanner](../src/main/deploy/pathplanner/) and edit its `paths` and `autos` in the PathPlanner app. |
| Add an autonomous action or chooser option | Edit [RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp). Named commands can be used as PathPlanner event markers. |
| Change a controller button | Edit `ConfigureBindings()` in [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp). |
| Change what an intake, conveyor, indexer, hood, or shooter command does | Edit [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). The command interfaces are in [Mechanisms.h](../src/main/include/season/mechanisms/Mechanisms.h). |
| Change mechanism CAN IDs, ratios, gains, limits, presets, or controller CAN rates | Edit the constants at the top of [Mechanisms.h](../src/main/include/season/mechanisms/Mechanisms.h). |
| Change drivetrain hardware settings or swerve odometry rate | Edit [DriveConfig.h](../src/main/include/season/drive/DriveConfig.h). |
| Change QuestNav or AprilTag camera mounts | Edit [VisionConfig.h](../src/main/include/season/vision/VisionConfig.h). |
| Change object-perception camera mounts, detection range, clustering, or update period | Edit [PerceptionConfig.h](../src/main/include/season/vision/PerceptionConfig.h). |
| Change the standard robot loop or controller ports | Edit [RobotControlConfig.h](../src/main/include/season/RobotControlConfig.h). |
| Change the shot table or shooting limits | Edit [Shooting.h](../src/main/include/season/shooting/Shooting.h). Shot calculations are in [Shooting.cpp](../src/main/cpp/season/shooting/Shooting.cpp). |
| Change AutoDrive or offensive planning settings | Start with the constants at the top of [AutoDrive.h](../src/main/include/season/drive/AutoDrive.h) or [OffensiveAutoDrive.h](../src/main/include/season/drive/OffensiveAutoDrive.h). |
| Change a simulation assumption | See [simulation configuration](#simulation) below. |

## Autonomous

PathPlanner routes and auto sequences live under [deploy/pathplanner](../src/main/deploy/pathplanner/). Edit them in PathPlanner so the app maintains their file format. The robot loads PathPlanner autos into the `Auto Choices` chooser.

[RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp) registers event markers such as `DeployIntake`, `SpinIntake`, and `AimedHubShot`, and adds the C++ chooser options. Use a registered name exactly in PathPlanner. Check the requirements of actions that run together: `AimedHubShot` owns the drivetrain and must not run alongside a path that also owns it. The [AutoDrive guide](autodrive-simulation.md) explains the existing choices.

## Mechanisms and constants

Start with the existing commands in [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). Commands that run motors while held need a stop or cleanup path when they end or are interrupted. For a new mechanism, use the shared MecoBase mechanism and motor IO interfaces rather than accessing a vendor controller directly in a command. [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp) constructs the season subsystems and binds commands to controls.

Put robot-specific settings at the top of the header for the subsystem or behavior they configure. Use `inline constexpr` for values that can be computed at compile time; configuration objects and calculations that need runtime evaluation use `inline const` or a function. Keep units and coordinate systems clear: rack travel is in metres, hood position is in output rotations, and flywheel speed is in rotations per second. Check current limits, motor direction, and travel limits against the actual robot before applying changes.

Some values are supplied by WPILib Preferences or the dashboard at runtime. Changing a header default does not necessarily replace an already-saved preference. See the [AutoDrive guide](autodrive-simulation.md) for the shooting and feed preferences and the [control timing guide](control-timing.md) for the standard loop and requested CAN, sensor, and input rates.

## Simulation

- [SimulationConfig.h](../src/main/include/season/simulation/SimulationConfig.h): fuel behavior, motor model, contact estimates, simulation timing, sensors, battery, and practice settings.
- [FuelSimulation.cpp](../src/main/cpp/season/simulation/FuelSimulation.cpp): robot-state inputs, transport motor loads, and simulation telemetry.
- [FuelModel.cpp](../src/main/cpp/season/simulation/FuelModel.cpp): fuel particle motion, collection, scoring, and model state transitions.
- [Simulation.h](../src/main/include/season/simulation/Simulation.h): public desktop simulation interfaces, including the fuel and loaded-motor model contracts.

Simulation settings are estimates; they do not establish physical calibration. See [simulation field](simulation-field.md) for the model and its limits.

## Check your change

Run `./gradlew build --max-workers=2` from the repository root, then use `./gradlew simulateNative` and exercise the affected controls or autonomous choice. A successful build checks compilation and the existing tests; a simulation check shows only simulated behavior. Before deploying or moving the physical robot, review the change with the team and verify the relevant hardware settings and safe operating conditions.
