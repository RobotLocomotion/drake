from __future__ import absolute_import

import numpy as np
from ._mathematicalprogram import *


def _NewContinuousVariables(self, *args, **kwargs):
    wrapper = self._NewContinuousVariables(*args, **kwargs)
    return np.array([wrapper[i] for i in range(len(wrapper))])

MathematicalProgram.NewContinuousVariables = _NewContinuousVariables
