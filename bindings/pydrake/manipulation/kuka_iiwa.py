"""Shim module that provides vestigial names for pydrake.manipulation.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.manipulation directly.

This module will be deprecated at some point in the future.
"""

from pydrake.manipulation import (
    ApplyDriverConfig,
    BuildIiwaControl,
    IiwaCommandReceiver,
    IiwaCommandSender,
    IiwaControlMode,
    IiwaDriver,
    IiwaStatusReceiver,
    IiwaStatusSender,
    ParseIiwaControlMode,
    get_iiwa_max_joint_velocities,
    kIiwaArmNumJoints,
    kIiwaLcmStatusPeriod,
    position_enabled,
    torque_enabled,
)
