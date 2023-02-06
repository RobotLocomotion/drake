# Jaco models

This folder contains models of Kinova Jaco robots, based originally on models
from Kinova (see [`LICENSE.TXT`](./LICENSE.TXT) for more information).

`urdf/j2n6s300*` are models related to the Jaco v2 6DOF non-spherical robot.

`urdf/j2s7s300.urdf` is a model of the full Jaco v2 7DOF spherical
robot using mesh collision geometry as originally supplied by Kinova.
The versions with `_arm` and `_hand` suffixes are the arm only and
hand only.

The versions with the `_sphere_collision` suffix use collision
geometry defined entirely by sphere primitives.  These models were
created by hand editing the urdf and verifying the results using a
visualization tool such as `//manipulation/util:show_model`.

In addition, some tags unsupported by Drake have been removed, to reduce the
burden of warning output. For URDF support details, see:
https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
