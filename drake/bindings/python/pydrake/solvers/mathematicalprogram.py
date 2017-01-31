from __future__ import absolute_import

import numpy as np
from ._pybind_mathematicalprogram import *
from ._pybind_mathematicalprogram import _VectorXDecisionVariable


def VectorXDecisionVariable(vars_as_ndarray):
    assert vars_as_ndarray.ndim == 1, "Only 1-D arrays are supported yet"
    wrapper = _VectorXDecisionVariable(vars_as_ndarray.size)
    for i in range(vars_as_ndarray.size):
        wrapper[i] = vars_as_ndarray[i]
    return wrapper


# Monkey-patching functions which return vectors of decision variables
# in order to convert those to numpy ndarray of decision variable wrappers
def _NewContinuousVariables(self, *args, **kwargs):
    wrapper = self._NewContinuousVariables(*args, **kwargs)
    return np.array([wrapper[i] for i in range(len(wrapper))])

MathematicalProgram.NewContinuousVariables = _NewContinuousVariables


def _NewBinaryVariables(self, *args, **kwargs):
    wrapper = self._NewBinaryVariables(*args, **kwargs)
    return np.array([wrapper[i] for i in range(len(wrapper))])

MathematicalProgram.NewBinaryVariables = _NewBinaryVariables


def _AddLinearCost(self, c, x):
    return self._AddLinearCost(c, VectorXDecisionVariable(x))

MathematicalProgram.AddLinearCost = _AddLinearCost

def _GetSolution(self, x):
    if isinstance(x, np.ndarray):
        return self._GetSolution(VectorXDecisionVariable(x))
    else:
        return self._GetSolution(x)

MathematicalProgram.GetSolution = _GetSolution

