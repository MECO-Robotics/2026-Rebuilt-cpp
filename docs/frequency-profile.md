# Fast control profile

The default robot loop remains 20 ms. Set `FRC_FAST_CONTROL=1` to opt into a separate 2 ms command-scheduler callback. The `TimedRobot` loop remains 20 ms for match-context, telemetry and simulation updates. The callback runs synchronously in the `TimedRobot` main loop; it does not create a worker thread.

Fast control requires a local MecoBase build with the fast-control API and the matching `MECO_FAST_CONTROL_API` application compile definition. If the runtime variable is set without that API build, startup fails with an explicit error. Public MecoBase coordinates and the default robot profile are unchanged.

With the opt-in enabled, `DriveConfig.controlPeriodSeconds` is 2 ms so field-relative velocity discretization uses the scheduler cadence. Desktop drivetrain simulation is externally scheduled once from the 20 ms simulation callback; a fast scheduler run cannot advance the 20 ms simulation clock. SPARK mechanism control runs at 10 ms; Kraken mechanism control runs at 4 ms on CAN FD and at 10 ms on classic CAN. Physics remains at 20 ms. SPARK feedback is requested at 100 Hz, Kraken mechanism feedback/native request resends at 250 Hz on FD or 100 Hz on classic, and drive odometry at 500 Hz on FD or 100 Hz on classic. Temperature and noncritical diagnostics are reduced where safe; SPARK reset/fault and follower-critical frames retain their existing rates. These are requested rates, not measured physical CAN delivery.

`/Simulation/FastControl/Samples` publishes one double array per fast callback:

`[sequence, scheduledFpgaUs, actualStartFpgaUs, finishFpgaUs, workUs, latenessUs, enabled]`

The array separates callback work from scheduling interval and lateness. It is diagnostic output and contributes some NT traffic. A 2 ms average interval or short desktop run does not establish hard real-time behavior, physical drivetrain response, CAN capacity, or safe robot operation.

The opt-in NT gamepad adapter remains a separate input setting (`FRC_SIM_NT_GAMEPAD=1`). Its `/Simulation/NTGamepad/Consumed` telemetry carries the NT source and consumption timestamps for input-to-command age; compare those in the NT clock domain rather than subtracting them from FPGA timestamps.

See [measured results and reproducible local build](frequency-results.md) for the baseline comparisons, hardware-validation limits and matching Base/Simulation development packages.
