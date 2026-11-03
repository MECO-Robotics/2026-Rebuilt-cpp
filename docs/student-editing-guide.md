# Where to make changes

Start with the task you want the robot to do. These files contain this season's robot-specific choices; the shared drive, mechanism, and motor IO implementations come from the vendordeps.

| I want to… | Edit here |
| --- | --- |
| Change an autonomous route or sequence | Open [PathPlanner](../src/main/deploy/pathplanner/) and edit its `paths` and `autos` in the PathPlanner app. |
| Add an autonomous action or chooser option | Edit [RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp). Named commands can be used as PathPlanner event markers. |
| Change a controller button | Edit `ConfigureBindings()` in [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp). |
| Change what an intake, conveyor, indexer, hood, or shooter command does | Edit [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). The command interfaces are in [Mechanisms.h](../src/main/include/season/mechanisms/Mechanisms.h). |
| Change mechanism CAN IDs, ratios, gains, limits, or presets | Edit [MechanismConfig.h](../src/main/include/season/mechanisms/MechanismConfig.h). |
| Change drivetrain settings or localization mounts | Edit [RobotConfiguration.h](../src/main/include/season/mechanisms/RobotConfiguration.h). |
| Change the shot table or shooting limits | Edit [ShootingConfig.h](../src/main/include/season/shooting/ShootingConfig.h). Shot calculations are in [Shooting.cpp](../src/main/cpp/season/shooting/Shooting.cpp). |
| Change AutoDrive or offensive planning settings | Start with [AutoDriveConfig.h](../src/main/include/season/drive/AutoDriveConfig.h) or [OffensiveConfig.h](../src/main/include/season/drive/OffensiveConfig.h). |
| Change a simulation assumption | See [simulation configuration](#simulation) below. |

## Autonomous

PathPlanner routes and auto sequences live under [deploy/pathplanner](../src/main/deploy/pathplanner/). Edit them in PathPlanner so the app maintains their file format. The robot loads PathPlanner autos into the `Auto Choices` chooser.

[RobotAutos.cpp](../src/main/cpp/season/auto/RobotAutos.cpp) registers event markers such as `DeployIntake`, `SpinIntake`, and `AimedHubShot`, and adds the C++ chooser options. Use a registered name exactly in PathPlanner. Check the requirements of actions that run together: `AimedHubShot` owns the drivetrain and must not run alongside a path that also owns it. The [AutoDrive guide](autodrive-simulation.md) explains the existing choices.

## Mechanisms and constants

Start with the existing commands in [SeasonCommands.cpp](../src/main/cpp/season/mechanisms/SeasonCommands.cpp). Commands that run motors while held need a stop or cleanup path when they end or are interrupted. For a new mechanism, use the shared MecoBase mechanism and motor IO interfaces rather than accessing a vendor controller directly in a command. [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp) constructs the season subsystems and binds commands to controls.

Put robot-specific settings in the relevant configuration header. Use `inline constexpr` for header constants that can be computed at compile time; some configuration objects and calculations need `inline const` instead. Keep units and coordinate systems clear: rack travel is in metres, hood position is in output rotations, and flywheel speed is in rotations per second. Check current limits, motor direction, and travel limits against the actual robot before applying changes.

Some values are supplied by WPILib Preferences or the dashboard at runtime. Changing a header default does not necessarily replace an already-saved preference. See the [AutoDrive guide](autodrive-simulation.md) for the shooting and feed preferences and the [control profile guide](frequency-profile.md) for deploy-time scheduler, CAN, and input settings.

## Simulation

- [FuelConfig.h](../src/main/include/season/simulation/FuelConfig.h): fuel capacity, transport, and simulated particle behavior.
- [RobotContactConfig.h](../src/main/include/season/simulation/RobotContactConfig.h): estimated bumper and chassis contact properties.
- [MotorModelConfig.h](../src/main/include/season/simulation/MotorModelConfig.h) and [RuntimeConfig.h](../src/main/include/season/simulation/RuntimeConfig.h): simulated motor, battery, sensor, and practice settings.
- [FuelSimulation.cpp](../src/main/cpp/season/simulation/FuelSimulation.cpp): season fuel behavior when the model itself needs to change.

Simulation settings are estimates; they do not establish physical calibration. See [simulation field](simulation-field.md) for the model and its limits.

## Check your change

Run `./gradlew build --max-workers=2` from the repository root, then use `./gradlew simulateNative` and exercise the affected controls or autonomous choice. A successful build checks compilation and the existing tests; a simulation check shows only simulated behavior. Before deploying or moving the physical robot, review the change with the team and verify the relevant hardware settings and safe operating conditions.
