# Control timing

The robot runs command decisions and mechanism control on the standard 20 ms `TimedRobot` loop. The same cadence is passed to the command scheduler, drivetrain configuration, and mechanism control calls.

Configure each rate in the subsystem header that owns it:

- Robot loop and command cadence: `kControlPeriod` in [RobotControlConfig.h](../src/main/include/season/RobotControlConfig.h). Both `TimedRobot` and the command scheduler use this value.
- Swerve odometry rates for CAN FD and classic CAN: `kSwerveOdometryFdHz` and `kSwerveOdometryClassicHz` in [DriveConfig.h](../src/main/include/season/drive/DriveConfig.h).
- Mechanism motor CAN control resend and status refresh rates: the `kSpark*Hz` and `kTalon*Hz` constants in [MechanismConfig.h](../src/main/include/season/mechanisms/MechanismConfig.h). [RobotPlatform.cpp](../src/main/cpp/season/hardware/RobotPlatform.cpp) selects the appropriate controller and bus rate.
- QuestNav update period: `kQuestSubscriptionPeriodSeconds` in [VisionConfig.h](../src/main/include/season/vision/VisionConfig.h).
- Object-detection subscription period: `kObjectDetectionPeriodSeconds` in [PerceptionConfig.h](../src/main/include/season/vision/PerceptionConfig.h).
- NT gamepad subscription and freshness: constants in [DriveBehaviorConfig.h](../src/main/include/season/drive/DriveBehaviorConfig.h).
- Desktop physics step: `kSimulationPeriodSeconds` in [SimulationConfig.h](../src/main/include/season/simulation/SimulationConfig.h).

The configured rates are requests to the libraries and devices; they do not prove delivered CAN frequency, signal age, or robot-loop timing. The robot currently uses a 20 ms command and mechanism loop. The TODO beside the control constants records that increasing the loop frequency requires end-to-end timing, motor IO, CAN load, and sensor transport validation. QuestNav measurement rate remains limited by its headset, and changing a subscriber period does not accelerate camera inference.

The [historical frequency results](frequency-results.md) describe an earlier experimental fast-control implementation that has since been removed. They are not a description of the current runtime.
