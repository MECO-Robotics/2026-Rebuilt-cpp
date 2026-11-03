# Robot documentation

- [Where to make changes](student-editing-guide.md): find the files for autos, mechanisms, controls, and constants.

- [AutoDrive](autodrive-simulation.md): controls, modes, and offensive behavior.
- [Practice match](practice-mode.md): start and troubleshoot a simulated match.
- [Simulation field](simulation-field.md): starting pose, fuel, contact settings, and troubleshooting.
- [Pilot controls](../README.md#pilot-controls): controller layout and bindings.

## Vendordep documentation

- [MecoBase AutoDrive](https://github.com/MECO-Robotics/MecoBase-cpp/blob/main/docs/autodrive.md) and [footprint configuration](https://github.com/MECO-Robotics/MecoBase-cpp/blob/main/docs/footprint-configuration.md).
- [MecoSimulation physics](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/jolt-simulation.md), [bumper contact](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/compliant-bumper-contact.md), and [practice controller](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/practice-mode.md).

## Preview the site

Install with `python3 -m pip install -r docs-requirements.txt`, then run `mkdocs serve` from the repository root. `mkdocs build` writes the site to `site/`. GitHub Actions builds the site and can publish it after Pages is configured to use GitHub Actions.

The [control timing guide](control-timing.md) describes the standard loop and where its requested rates are configured. Use the [NT gamepad guide](nt-gamepad.md) for the laptop controller publisher. [Historical timing measurements](frequency-results.md) document an earlier fast-control experiment and do not describe the current runtime.
