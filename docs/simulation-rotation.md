# Robot won't turn in simulation

First check whether the robot is upright. A simulated overturned robot can show wheel motion and a turn request without rotating its chassis because its wheels have no ground support.

- Compare the requested turn, wheel motion, and physical orientation in your dashboard.
- Use the pilot's **Back/View** button in teleop to reset the simulated scene.
- If inputs update slowly, follow the [latency guide](simulation-latency.md).
- A successful desktop reset does not establish that physical steering is calibrated.

See the [reset binding](../src/main/cpp/RobotContainer.cpp), [robot geometry](../src/main/include/season/RobotGeometry.h), and [original investigation](reference/simulation-rotation.md).
