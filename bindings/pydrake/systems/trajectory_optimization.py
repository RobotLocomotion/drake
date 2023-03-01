"""Deprecated aliases to planning resources."""

from pydrake.common.deprecation import _warn_deprecated

# Note: This imports more than just the trajectory optimization symbols
# (because planning is flat). If this is too generous in exposing planning
# symbols in this module, we'll need to enumerate all the former members of
# this module explicitly.
from pydrake.planning import *
# Historically, TimeStep belonged to the module; pull it out to maintain
# backwards compatibility until completed deprecation.
TimeStep = DirectTranscription.TimeStep

_warn_deprecated(
    "Please import the trajectory optimization from pydrake.planning instead "
    f"of the deprecated {__name__} submodule.",
    date="2023-05-01", stacklevel=3)
