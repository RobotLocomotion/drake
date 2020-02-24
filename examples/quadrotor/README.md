
The Quadrotor - an underactuated aerial vehicle
===============================================

This directory contains models and a plant for a Quadrotor.

You can try out sample programs; each has its own overview atop the file:

 - `run_quadrotor_dynamics`
 - `run_quadrotor_lqr`

Historical note: IRIS mixed-integer convex planning
---------------------------------------------------

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation of the Quadrotor you may use
this tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Quadrotor

In particular, the IRIS mixed-integer convex planning examples have not yet
been re-implemented in the new framework (Drake issue #6243).  To view that
algorithm and example, you'll need to consult the above link.
