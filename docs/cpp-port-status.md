# C++ port status

This project is a native C++ implementation of the 2026 robot. Shared drive, motor IO, and command features come from MecoBase; season wiring and behavior live here.

## What is here?

- Swerve drive, intake, shooter, autonomous commands, and controller bindings: [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp).
- Shooting and feed readiness: [Shooting.cpp](../src/main/cpp/season/Shooting.cpp) and [SeasonCommands.cpp](../src/main/cpp/season/SeasonCommands.cpp).
- Limelight and QuestNav inputs, plus desktop-simulated localization.
- Desktop field, fuel, and drivetrain simulation: [simulation guide](simulation-field.md).
- Test-mode system checks: [SystemCheckSetup.cpp](../src/main/cpp/season/SystemCheckSetup.cpp).

## What still needs care?

- Native WPILib logging replaces Java-only AdvantageKit features; this is not a Java log replay implementation.
- Source constants and simulated results are not hardware calibration. Commission motors, sensors, localization, controls, and mechanisms on the real robot.
- A successful build or simulator run does not establish full Java parity, safe field behavior, or a successful deployment.

See the [port contract](port-contract.md) and [historical port and validation record](reference/cpp-port-status.md) for detailed differences and evidence.
