# Drive acceleration and braking in simulation

The simulator models the motor, drivetrain, robot mass, and wheel contact so drive requests produce motion instead of instant speed changes.

- The desktop configuration uses an estimated 67.6 kg ready-to-drive mass and a 5.89:1 drive ratio.
- It uses the Kraken X60 FOC motor model and the configured current limits. Those values affect acceleration and braking.
- Simulated traction, inertia, and response are estimates. Do not use a simulation pass as proof of real stopping distance or hardware gains.

Read the [drive configuration](../src/main/cpp/RobotContainer.cpp), [field model](simulation-field.md), and [detailed dynamics record](reference/simulation-dynamics.md).
