# Port contract

These are the rules for moving robot behavior into this C++ project:

- Keep reusable robot infrastructure in MecoBase; keep 2026 rules, wiring, and policy in this project.
- Preserve the Java sources and use the pinned 2026 WPILib and vendor APIs.
- Use metres for field positions and linear joints; use rotations and rotations per second for mechanism angles and speeds.
- Give commands clear subsystem ownership and stop behavior on interruption.
- Test the behavior that matters, and distinguish build, simulation, and physical-robot evidence.

Start with [robot configuration and bindings](../src/main/cpp/RobotContainer.cpp) and [mechanism interfaces](../src/main/include/season/Mechanisms.h). The [historical contract](reference/port-contract.md) records the original migration assumptions and sources.
