# Simulated battery

The simulator estimates how motor current affects the shared bus voltage. This helps the drivetrain and mechanisms see the same power supply.

```mermaid
flowchart LR
  Motors[Simulated motor current] --> Battery[Battery estimate]
  Battery --> Voltage[Next loop's bus voltage]
  Voltage --> Motors
```

- The model starts from 12 V and uses an estimated 20 mΩ internal resistance.
- It uses battery-side supply current, not a direct sum of stator currents.
- The next voltage is smoothed to avoid numerical oscillation. These values are estimates, not measurements of our battery.
- The desktop loop calls the helper once; this does not change the physical robot's power system.

See [Robot.cpp](../src/main/cpp/Robot.cpp), the [battery smoke test](../scripts/battery_sag_smoke.py), and the [detailed battery record](reference/simulation-battery.md).
