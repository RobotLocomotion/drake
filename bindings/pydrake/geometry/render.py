"""Deprecated aliases to pydrake.geometry.render resources."""

from pydrake.common.deprecation import _warn_deprecated
from pydrake.geometry import *

_warn_deprecated(
    "Please import the render-related symbols from pydrake.geometry instead "
    f"of the deprecated {__name__} submodule.",
    date="2023-08-01", stacklevel=3)
