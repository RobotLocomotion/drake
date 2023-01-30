"""Shim module that provides vestigial names for pydrake.examples.
Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.examples directly.
"""

from pydrake.common.deprecation import _warn_deprecated

from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotParams,
    AcrobotPlant,
    AcrobotSpongController,
    AcrobotState,
    SpongControllerParams,
)

_warn_deprecated(
    "Please import from the pydrake.examples module directly, instead of the "
    f"deprecated {__name__} submodule.",
    date="2023-05-01", stacklevel=3)
