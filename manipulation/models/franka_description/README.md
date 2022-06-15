This folder contains a drake-compatible model of the Franka Emika Panda arm.

The model differs from the original ROS model and the real robot. In this model
the fingers are independently actuated, rather than using <mimic> tag, which
Drake does not yet support.

In addition, some tags unsupported by Drake have been removed, to reduce the
burden of warning output. For URDF support details, see:
https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
