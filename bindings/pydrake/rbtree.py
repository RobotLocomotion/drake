from __future__ import absolute_import

# N.B. This is a shim that will soon be removed.
# TODO(eric.cousineau): On 2/15/2017, when this module is imported, print a
# deprecation warning.
# TODO(eric.cousineau): On 3/31/2017, remove this module.

from .multibody.rigid_body_tree import *

# TODO(eric.cousineau): Remove this direct import and ask users to import
# pydrake.parsers.PackageMap instead
from .multibody.parsers import PackageMap

# TODO(eric.cousineau): remove these direct imports and leave the constants
# inside the FloatingBaseType enum class (#4950).
kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion
