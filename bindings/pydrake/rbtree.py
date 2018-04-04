from __future__ import absolute_import

import warnings

from .util.deprecation import DrakeDeprecationWarning

# N.B. This is a shim that will be removed soon-ish.

# TODO(eric.cousineau): On 06/02/2018 (after the underactuated course is done),
# post a warning that this module will be deprecated.
# On 07/11/2018 (after 1.5 months), remove the module.

from .multibody.rigid_body_tree import *

# TODO(eric.cousineau): Remove this direct import and ask users to import
# pydrake.parsers.PackageMap instead
from .multibody.parsers import PackageMap

# N.B. `stacklevel` is a bit difficult here, because it could be triggered by
# `import pydrake.rbtree` (module import) or `from pydrake import rbtree`
# (ModuleShim).
warnings.warn(
    "`pydrake.rbtree` is deprecated. Please use " +
    "`pydrake.multibody.rigid_body_tree` and `pydrake.multibody.parsers` " +
    "instead.",
    category=DrakeDeprecationWarning)

# TODO(eric.cousineau): remove these direct imports and leave the constants
# inside the FloatingBaseType enum class (#4950).
kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion
