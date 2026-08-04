# Manipulation

This package includes manipulation utilities for interfacing with simulation
and hardware, including models, LCM middleware translators, etc.

## Conventions

<!--
Per discussion in https://github.com/RobotLocomotion/drake/issues/13374 and related threads.
-->

Note that (at present) some hardware drivers may report different sign
conventions. These are represented directly in the LCM messages, and the onus
is presently on the user to apply appropriate sign flips.

For hardware and simulation drivers, the following conventions are established:

- Translation in meters, Angles in radians,
- Torque in Newton-meters, Force in Newtons
- Positive absolute force / torque corresponds to positive linear / angular acceleration
- Commanded torque and measured torque should be roughly equivalent
- External forces / torque should be positive when controller deflection[^1]
  is positive.
- By association, external spatial forces (wrenches) should follow the same
  sign convention as external torques.

[^1]: Say we have a single axis, and we have a stiffness controller
(a PD controller plus gravity compensation). This controller is targeting a
constant position (zero velocity). If an external force causes the axis to move
in a positive direction, then the external force should (of course) be
considered positive. While obvious, this is important to highlight especially in
the presence of a feedback controller, which will increase its effort to
counteract said external force. Depending on how vendors report this effort,
users may need to flip the sign.

The following swaps are needed for measured torque and external torque for the given
robot vendors:

| Hardware Robot | Measured Absolute Torque | Measured External Torque |
|------|---------|---------|
| IIWA | negated | correct |
| Jaco | correct | negated |
| Panda | correct | negated |

Note that the *values in the LCM messages* will have these conventions.
If you are authoring an in-process simulation diagram, both of these conventions should
be "correct".
