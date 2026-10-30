# Simulation field and fuel

The desktop simulator builds a simplified REBUILT field around the robot. The drive adapter owns the physics world; the season code chooses the starting pose and updates fuel.

```mermaid
flowchart LR
  Physics[Drive physics] --> Truth[Physical robot pose]
  Truth --> Sensors[Simulated localization]
  Truth --> Fuel[Fuel contacts and launch]
  Sensors --> Odometry[Estimated pose]
```

## What is modeled?

- Walls, hubs, bump plates, trench supports and arms, and simplified towers and depots.
- Fuel as particles, including approximate field and robot contacts. Escaped fuel returns to the carpet; fuel is not a collection of Jolt rigid bodies.
- A low chassis and bumper envelope. Decorative geometry and full mechanism collisions are omitted.

The scene is an approximation, not official CAD or proof of field clearance. See the [detailed field record](reference/simulation-field.md) for dimensions, sources, and tests.

## Starting and resetting

- Default blue spawn: approximately `(3.65, 0.64 m, 0°)`; red is mirrored to approximately `(12.89, 7.43 m, 180°)`.
- A selected auto can request its initial pose. In simulation, the physical body and odometry are reset together; starts intersecting field geometry are rejected.
- `/SmartDashboard/Simulation/AutoStartPose` reports `applied`, `blocked-by-field`, or `not-requested`.
- In teleop, pilot **Back/View** resets the scene. Pilot **Start** resets driver forward, not the field pose.
- The simulator waits for feedback to settle after a reset. Physical truth, camera observations, and estimated odometry remain distinct.

## Read the code

- [Scene setup and pose resets](../src/main/cpp/RobotContainer.cpp)
- [Field helpers](../src/main/cpp/season/SimulationField.cpp)
- [Fuel simulation](../src/main/cpp/season/FuelSimulation.cpp) and [model interface](../src/main/include/season/Simulation.h)
- [Auto-start pose smoke check](../scripts/auto_start_pose_smoke.py)
