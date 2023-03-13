"""Shim module that provides vestigial names for pydrake.manipulation.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.manipulation directly.

This module will be deprecated at some point in the future.
"""

from pydrake.manipulation import (
    ApplyDriverConfig,
    BuildSchunkWsgControl,
    GetSchunkWsgOpenPosition,
    MakeMultibodyForceToWsgForceSystem,
    MakeMultibodyStateToWsgStateSystem,
    SchunkWsgCommandReceiver,
    SchunkWsgCommandSender,
    SchunkWsgController,
    SchunkWsgDriver,
    SchunkWsgPositionController,
    SchunkWsgStatusReceiver,
    SchunkWsgStatusSender,
)
