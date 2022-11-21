"""Shim module that provides vestigial names for pydrake.examples.
Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.examples directly.
This module will be deprecated at some point in the future.
"""

from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotParams,
    AcrobotPlant,
    AcrobotSpongController,
    AcrobotState,
    SpongControllerParams,
)
