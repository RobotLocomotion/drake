"""Deprecated aliases to planning resources."""

from pydrake.common.deprecation import _warn_deprecated

# Note: This imports more than just the differential ik symbols
# (because inverse_kinematics has more than just that). If this is too generous
# in exposing differential id symbols in this module, we'll need to enumerate
# all the former members of this module explicitly.
from pydrake.multibody.inverse_kinematics import *

_warn_deprecated(
    "Please import the differential ik API from "
    "pydrake.multibody.inverse_kinematics instead of the deprecated "
    f"{__name__} submodule.",
    date="2023-05-01", stacklevel=3)
