# Robot documentation

- [AutoDrive](autodrive-simulation.md): controls, modes, and offensive behavior.
- [Practice match](practice-mode.md): start and troubleshoot a simulated match.
- [Simulation field](simulation-field.md): starting pose, fuel, contact settings, and troubleshooting.
- [Pilot controls](../README.md#pilot-controls): controller layout and bindings.

## Vendordep documentation

- [MecoBase AutoDrive](https://github.com/MECO-Robotics/MecoBase-cpp/blob/main/docs/autodrive.md) and [footprint configuration](https://github.com/MECO-Robotics/MecoBase-cpp/blob/main/docs/footprint-configuration.md).
- [MecoSimulation physics](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/jolt-simulation.md), [bumper contact](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/compliant-bumper-contact.md), and [practice controller](https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/docs/practice-mode.md).

## Preview the site

Install with `python3 -m pip install -r docs-requirements.txt`, then run `mkdocs serve` from the repository root. `mkdocs build` writes the site to `site/`. GitHub Actions builds the site and can publish it after Pages is configured to use GitHub Actions.
