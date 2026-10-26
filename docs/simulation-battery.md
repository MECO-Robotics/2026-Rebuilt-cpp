# Shared battery simulation

Desktop `Robot::SimulationPeriodic` calls `meco::sim::UpdateBatterySupply` after
physics has completed. Drivetrain drive/steer motors and mechanism SimMotorIO
plants register battery-side supply current with the same runtime. Their sum
sets the next period's shared bus voltage and roboRIO input voltage/current:

`targetV = max(0, 12 - 0.020 * totalSupplyCurrentAmps)`

`nextBusV = 0.5 * (previousBusV + targetV)`

The defaults are the [WPILib BatterySim estimates](https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1_battery_sim.html),
not measurements of our battery. Stator current is not directly summed as battery
current. Phoenix receives the reduced bus voltage; mechanism motor outputs are
also clamped to that bus. Real-robot behavior is unchanged.

This is explicit feedback at 20 ms: the completed period's current determines
target voltage for the next period. One half-weight numerical relaxation step
prevents the alternating high/low supply cycle observed with direct feedback.
It is not a calibrated battery capacitance or electrochemical time constant.
It adds no additional physics steps or controller
threads. The helper is maintained upstream in MecoSimulation's WPILib adapter;
a matching header is included locally until a compatible vendordep is published.
No ABI layout changes are required.

Telemetry:

- `/SmartDashboard/SimulationPower/SupplyCurrentAmps`
- `/SmartDashboard/SimulationPower/BusVoltage`
- `/Simulation/PowerSamples`: atomic array `[NT timestamp microseconds, supply amps, bus volts]`.

`BatterySupplyConfig` allows open-circuit voltage, resistance, and an auxiliary
constant current; this robot currently uses defaults with zero auxiliary current.
Electronics/pneumatics loads, individual wiring drops, battery discharge,
regeneration and breaker heating are not modeled. The helper sets roboRIO input
voltage; it does not implement independent brownout motor-output suppression.
This remains a delayed approximation, not an implicit electrical circuit solver.
Extreme loads and brownout recovery require separate validation.

Validation uses the native shared-runtime load/sag/recovery regression and the
actual HAL joystick/mechanism path:

```bash
FRC_SIM_FIELD_COLLIDERS=0 ./scripts/sim_headless.sh
# In another terminal with the existing RobotPy/NT/websocket environment:
python scripts/battery_sag_smoke.py --output artifacts/battery-sag-validation.json
```

The smoke disables the robot on exit. It checks drivetrain and mechanism sag,
the recorded current/voltage equation, and recovery after disabling. Existing
azimuth and braking evidence predates battery sag and is not acceptance evidence
for the updated electrical model.

## Recorded validation

[Desktop evidence](../artifacts/battery-sag-validation.json): all four native
build targets and 28 tests in each desktop configuration passed. The shared-load
regression covers sag, recovery and roboRIO/plant agreement. A high-conductance
load regression converges to its 4 V circuit equilibrium; direct unrelaxed
feedback would alternate between 0 and 12 V.

The actual joystick/intake capture measured minimum bus voltage of 9.46 V during
drive acceleration and 9.79 V with intake mechanisms running; disabling recovered
to 12 V. The largest adjacent voltage step over the last half-second of steady
drive fell from 3.55 V with direct feedback to 0.0084 V with relaxation; combined
drive/intake fell from 4.74 V to 0.0913 V. The phases occur sequentially at different
speeds, so their peak currents are not an additive load comparison.

The existing drive smoke, using the documented season arguments, passed all
checks except the azimuth-travel bound during direction changes. That tracking
limitation remains. The older mechanism smoke could not complete because its
expected voltage topics are absent; its acceptance is not claimed. The dedicated
battery smoke did exercise the actual intake controls and shared current path.
