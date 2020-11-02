
Historical note: MATLAB-based examples
======================================

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation of various examples, you may
use this tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples

In particular, these examples have not yet been re-implemented in the new
framework:

 - Airplane2D
 - CartPole
 - DubinsCar
 - FurutaPendulum
 - Glider
 - HolonomicDrive
 - Hubo
 - IRB140
 - KneedCompassGait
 - Manipulator2D
 - PlanarMonopodHopper
 - PlanarNLink
 - Quadrotor2D
 - SimpleDoublePendulum
 - SimplePulleys
 - SpringLoadedInvertedPendulum
 - TwoWheeledInvertedPendulum
 - UnderwaterAcrobot
 - Wingeron
 - ZMP
 - grasping
 - mass_spring_damper
 - valkyrie [1]

To view the original implementation of these examples, you'll need to consult
the above link.

[1] To view or use the final C++ implementation of Valkyrie based on
RigidBodyTree (removed as of 2019), you may use this commit:

https://github.com/RobotLocomotion/drake/tree/v0.11.0/examples/valkyrie