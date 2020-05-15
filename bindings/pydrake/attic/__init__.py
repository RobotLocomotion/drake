"""
Modules in `pydrake.attic` are deprecated, and will be removed from Drake on or
after 2020-09-01. This includes RigidBodyTree and RigidBodyPlant and their
visualization, sensors for RigidBodyPlant such as accelerometer, gyro, camera,
etc., inverse kinematics and inverse dynamics based on RigidBodyTree, and
SimDiagramBuilder and WorldSimTreeBuilder based on RigidBodyTree. Developers
should use the replacement MultibodyPlant family instead. See
https://github.com/RobotLocomotion/drake/issues/12158 for details.
"""

import warnings


# We use DeprecationWarning (not DrakeDeprecationWarning) for the attic to
# avoid causing fail-fast errors in our unit tests.
warnings.warn(
    __doc__.replace('\n', ' ').strip(),
    category=DeprecationWarning, stacklevel=2)
