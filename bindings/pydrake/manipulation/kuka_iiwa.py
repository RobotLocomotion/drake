"""Shim module that provides vestigial names for pydrake.manipulation.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.manipulation directly.
"""

from pydrake.common.deprecation import _warn_deprecated

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

_warn_deprecated(
    "Please import from the pydrake.manipulation module directly, instead of "
    f"the deprecated {__name__} submodule.",
    date="2023-11-01", stacklevel=3)
