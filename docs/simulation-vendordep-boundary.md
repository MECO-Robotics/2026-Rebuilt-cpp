# Simulation vendordep boundary

| Owner | Responsibilities |
| --- | --- |
| MecoBase | Commands, staged system checks, motor IO, hardware configuration and Jolt-free shared motor runtime |
| MecoSimulation | Jolt/world, rigid wheel, configurable rigid pan/backing and compliant bumper contact, load-dependent azimuth, Phoenix feedback bridge, battery and diagnostics helpers, field geometry, footprint clearance, tuning, ideal vision observations and prescribed-body particle contacts |
| This robot | Hardware dimensions/mass/current settings, controls, commands, robot-specific fuel behavior, legal starting-pose policy and telemetry presentation |

The robot consumes Simulation public headers and its combined static archive.
There are no local `base/sim` or `meco/sim` headers, copied physics translation
units, Jolt downloads or private library-layout compile flags. Native tests link
the packaged plant. `PhysicsTruth()` provides full physical orientation and
module contact data; reading chassis tilt does not intercept vendor feedback.

Battery and Driver Station packet helpers are explicitly called once by the
application's simulation loop. Neither adds another physics owner. The latency
probe wrapper supplies real-robot no-ops because Simulation is desktop-only.
The shared field map belongs to the 2026 package; spawn and chassis dimensions remain specific to this robot. The shared clearance
helper accepts these inputs and fails closed for unsupported collider geometry.

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

The historical 2026.0.9 [validation record](../artifacts/vendordep-compartmentalization-2026.0.9.json)
records all four clean Linux/Windows consumer runs, the robot's 28 tests per desktop
configuration, nine field checks, and the shared battery smoke. Public-package
verification is recorded separately from staged local-Maven checks.

The 2026.0.10 bump contact release and joystick checks are recorded in
[bump-contact-validation-2026.0.10.json](../artifacts/bump-contact-validation-2026.0.10.json).

Local development-package setup is documented in [contact configuration](compliant-bumper-contact.md). Linux local-package validation does not establish Windows or public-release acceptance.

Localization uses MecoBase's public `VisionSystem`, `InjectedVisionIO`, Limelight
and QuestNav factories. Season code composes these sources and supplies the
installed camera transforms and an ideal truth/visibility model; it copies no
vendor protocol, driver, private header or estimator implementation. MecoSimulation
remains the sole physics owner. The shared ideal-observation helper applies field and height bounds; the season
selects camera fix and alignment policy. Sensor noise, latency and wire protocol
remain outside that ideal model.

During the staged boundary migration, a development MecoBase package may provide
`base/season2026/MatchContext.h`, and a development MecoSimulation package may
provide `meco/sim/ParticleMotion.h`. The robot uses those headers when available
and otherwise keeps its existing local calculations for the published versions.
The motion helper does not step the physics world or own game rules. Robot-specific
fuel collection, launch configuration, scoring, and telemetry stay here. Remove
the compatibility fallbacks only after adopting published versions containing
both APIs and validating a clean consumer.
