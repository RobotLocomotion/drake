from __future__ import absolute_import, division, print_function

import numpy as np

from ._pydrake_rbtree import *

# TODO: remove these direct imports and leave the constants
# inside the FloatingBaseType enum class
# Ref #4950
kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion
