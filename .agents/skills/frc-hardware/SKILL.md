---
name: frc-hardware
description: Identify, configure, commission, or diagnose FRC robot hardware, including motor controllers, sensors, CAN, power, roboRIOs, and vision devices. Use for physical-device configuration and measured hardware behavior; use frc-base-project for C++ architecture changes.
---

# FRC hardware

Read the checkout's `AGENTS.md`, hardware configuration, and relevant port-status documents. Base constants are examples, not commissioned settings. Keep actual device IDs, offsets, ratios, limits, and calibration evidence in the season project. This skill does not itself authorize device writes, imaging, deployment, or motion; honor the scope already authorized by the user without asking again for covered actions.

## Identify before changing

- Establish the exact device model/revision, bus or interface, firmware, project season, and pinned vendor version. Distinguish source configuration, observed device state, and intended state. Record only details relevant to the task; do not invent values from a similar robot.
- Use official instructions for that model and version. When available, use season-matched FRC reference tools. Do not generalize connectors, boot modes, images, or recovery procedures across roboRIO generations or Limelight models. For example, roboRIO 2 uses a microSD image rather than direct roboRIO Imaging Tool imaging.
- Start with read-only discovery, faults, logs, and configuration export where supported. Trace symptoms through power, physical wiring, device discovery, communications, configuration, then control behavior; choose checks supported by the symptom rather than performing every check.

## Configuration and diagnosis

- Match each controller/sensor to its physical mechanism, configured bus and ID. Resolve address conflicts using the vendor's rules for that bus/device type. Inspect CAN topology, termination, connection quality, faults, and utilization before treating missing or stale signals as a software tuning problem. Measure resistance only with power removed and according to the hardware procedure.
- Verify power distribution, polarity, connections, and supply voltage under the relevant load. Distinguish supply and stator current limits; derive limits from the actual motor, controller, wiring, mechanism, and applicable season rules. Do not raise limits or disable protections merely to hide a fault.
- Establish an explicit known configuration for settings the program owns. Check apply/read status and relevant readback; bound retries and stop on persistent failure. Understand full-object defaults and persistent vendor settings before applying partial groups. Avoid repeated flash writes in periodic loops, and do not assume a successful API call proves correct physical behavior.
- After replacing a sensor, compare feedback-source selection, remote sensor ID, direction, and absolute calibration with the intended configuration; a discovered device is not necessarily the feedback source used by the controller.
- For network issues, isolate link, address/subnet, route, name resolution, and service reachability. Prefer targeted checks over blanket firewall/VPN changes. Record and restore any temporary network settings.

## Commission only the intended mechanism

- Before an authorized motion test, establish clearance, mechanical support for gravity-loaded axes, a local operator able to disable, and a working stop path. Use a bounded low-energy command appropriate to the mechanism, one axis at a time. A software stop or brake mode is not a mechanical support.
- Verify positive motor direction and sensor direction together. Check rotor/output ratios, encoder range and wrap, absolute offset, linear conversion, and reference position against a physical observation before closing the loop. Validate limit-switch polarity, homing, travel bounds, and reset behavior; soft limits depend on a valid coordinate reference.
- Check release, interruption, disable, and relevant fault responses before increasing speed/load. Stop on unexpected direction, sensor disagreement, limit violation, loss of control, or abnormal current/temperature. Diagnose before retrying. Tune only after polarity, feedback, ratios, and limits are established; do not mask these faults with gains.

## Firmware, imaging, and evidence

For an authorized update, verify model/season compatibility, preserve recoverable configuration, and identify the exact device or removable target by stable identity and capacity before writing. Use the official tool/procedure and its verification result; propagate write/read/decompression failures rather than reporting success from partial output. Do not substitute a prerelease or latest release for a required version. Afterward, confirm the booted firmware/image and relevant settings on the device.

Record the tested configuration, software revision, measurements, pass/fail criteria, and remaining uncertainty in existing project documentation or artifacts as appropriate. Separate static review, simulation, bench tests, and loaded robot operation. Follow the repository's less-is-more test guidance: a focused regression or commissioning observation should protect a distinct failure mode. Stop tests with the robot disabled and account for stored energy or position holding before leaving it unattended.

Consult these official starting points only for the task at hand, checking their season/model applicability:
- [WPILib CAN wiring](https://docs.wpilib.org/en/stable/docs/hardware/hardware-basics/can-wiring-basics.html)
- [roboRIO 2 imaging](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
- [Phoenix configuration semantics](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html)
