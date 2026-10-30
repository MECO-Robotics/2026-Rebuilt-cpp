> Historical detail: this record may describe earlier versions or experiments. Start with the [current guide](../compliant-bumper-contact.md).

# Season contact configuration

The robot now uses the MecoSimulation `2026.0.14` vendordep in
Full3D. `RobotContacts()` in `RobotContainer.cpp` owns the construction estimates:
2-inch belly-pan clearance, 1/8-inch pan, two 2.75-inch noodles per side and
3/4-inch by 5-inch plywood with its bottom flush at the user-selected 1/2 inch above carpet.
Its pressure curve, damping and tangential shear modulus (G = E/2) are estimates,
not calibration.
The season sets the foam damping ratio to 1.2 (up from 0.8) to dissipate more
impact energy and reduce rebound, with the same compression pressure curve.
This is simulation tuning; its impact response has not been calibrated to hardware.
The rigid plywood and solid pan can still high-center the robot.

See the upstream [contact model](../../../MecoSimulation-cpp/docs/compliant-bumper-contact.md)
for the solver, assumptions and telemetry. `FieldSimulation/*` reports effective
settings at startup. Contact loads and compression are published at 50 Hz without
`FRC_SIM_LATENCY_PROBE`; legacy planar mode retains rigid contact.

## Package setup

The normal build resolves version `2026.0.14` from public Maven. The season links
packaged public headers and one combined archive; it does not compile upstream
sources or override headers.

For local upstream development, rebuild both Linux configurations and explicitly
select the local package. If its version differs from the checked-in descriptor,
provide its version explicitly with `-PmecoSimulationVersion`:

```sh
../MecoSimulation-cpp/scripts/build_local_contact_package.sh
./gradlew build -PmecoSimulationMaven=../MecoSimulation-cpp/build/contact-package/maven --max-workers=2
```

The helper builds both CMake physics configurations and both Gradle adapter
configurations, stages one combined archive per configuration, records dirty-tree
provenance, and verifies the Linux package. It never publishes a release.

Use the field smoke driver against a task-owned simulator to record wheel unload,
foam compression, rigid contacts and complete crossing separately. Acceptance
requires complete upright full-stick crossing in the lane, then settled
wheel support. Contact/lift alone does not pass. Historical release artifacts remain
unchanged. Initial development results are in `artifacts/compliant-contact-validation.json`;
full-speed diagnosis and three consecutive passing runs are recorded in
[`bump-full-stick-validation.json`](../../artifacts/bump-full-stick-validation.json).
Peak tilt was 14.98–15.08 degrees, with settled wheel support after each crossing.
This establishes repeatability for the tested path, not measured hardware behavior.
