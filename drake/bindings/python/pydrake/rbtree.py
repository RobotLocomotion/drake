from __future__ import absolute_import, division, print_function

from ._pydrake_rbtree import *

# TODO: remove these direct imports and leave the constants
# inside the FloatingBaseType enum class
# Ref #4950
kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion
