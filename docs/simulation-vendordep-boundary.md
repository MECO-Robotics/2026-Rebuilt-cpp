# Simulation vendordep boundary

| Owner | Responsibilities |
| --- | --- |
| MecoBase 2026.2.1 | Commands, motor IO, hardware configuration and Jolt-free shared motor runtime |
| MecoSimulation 2026.0.9 | Jolt/world, rigid wheel and bumper contact, load-dependent azimuth, Phoenix feedback bridge, shared battery helper, simulation diagnostics and reusable 2026 field collider geometry |
| This robot | Hardware dimensions/mass/current settings, controls, commands, robot-specific fuel behavior, legal starting-pose policy and telemetry presentation |

The robot consumes Simulation public headers and its combined static archive.
There are no local `base/sim` or `meco/sim` headers, copied physics translation
units, Jolt downloads or private library-layout compile flags. Native tests link
the packaged plant. `PhysicsTruth()` provides full physical orientation and
module contact data; reading chassis tilt does not intercept vendor feedback.

Battery and Driver Station packet helpers are explicitly called once by the
application's simulation loop. Neither adds another physics owner. The latency
probe wrapper supplies real-robot no-ops because Simulation is desktop-only.
The shared field map belongs to the 2026 package; spawn and chassis-clearance
policy remain specific to this robot.

A release requires matching public headers/archives, clean source provenance,
Linux and Windows Debug/Release package consumers, and public Maven verification.
The package verifier rejects Base-owned headers in the Simulation archive.
Staged local-Maven validation is distinct from a published release. The historical
physics/drive evidence retains its original revision and limitations; packaging
does not resolve the known azimuth tracking failures.

For concurrent desktop validation, `FRC_SIM_NT_PORT=5811` starts this robot's
NetworkTables server on loopback port 5811. Use `--nt-port 5811` with the field
and battery smoke drivers. The default remains 5810. Confirm that the HAL websocket
and telemetry server belong to the same process; results mixing two simulators
are invalid. The evidence helper checks listening socket owners and rejects
mismatched endpoints before sending input. This option does not alter or stop
another simulator.

The [validation record](../artifacts/vendordep-compartmentalization-2026.0.9.json)
records all four clean Linux/Windows consumer runs, the robot's 28 tests per desktop
configuration, nine field checks, and the shared battery smoke. Public-package
verification is recorded separately from staged local-Maven checks.
