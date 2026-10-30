# Simulated bumper contact

The Full3D simulator models compressible bumper foam, a rigid plywood backing, and a solid belly pan. The robot's dimensions and material assumptions are set in [RobotContainer.cpp](../src/main/cpp/RobotContainer.cpp).

- The current setup uses an estimated 2-inch pan clearance and a 0.5-inch bumper floor gap.
- Foam stiffness and damping are estimates. The model can still high-center on the rigid backing or pan.
- The simulation publishes contact information; it does not establish physical impact performance or rule compliance.
- The physics implementation comes from the pinned [MecoSimulation vendordep](../vendordeps/MecoSimulation.json).

See the [detailed contact record](reference/compliant-bumper-contact.md) for assumptions and earlier package-validation results. The [older bumper guide](bumper-contact.md) explains the previous rigid model.
