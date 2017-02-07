from __future__ import absolute_import

import numpy as np
from ._pydrake_mathematicalprogram import *
from ..wrapperutils import wrap, unwrap


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
    return unwrap(Variable, wrapper)

MathematicalProgram.NewContinuousVariables = _NewContinuousVariables


def _NewBinaryVariables(self, *args, **kwargs):
    wrapper = self._NewBinaryVariables(*args, **kwargs)
    return unwrap(Variable, wrapper)

MathematicalProgram.NewBinaryVariables = _NewBinaryVariables


def _AddQuadraticCost(self, Q, b, vars):
    wrapper = wrap(VectorXDecisionVariable, vars)
    return self._AddQuadraticCost(Q, b, wrapper)

MathematicalProgram.AddQuadraticCost = _AddQuadraticCost


def _GetSolution(self, x):
    if isinstance(x, np.ndarray):
        return self._GetSolution(wrap(VectorXDecisionVariable, x))
    else:
        return self._GetSolution(x)

MathematicalProgram.GetSolution = _GetSolution


def _variables(self):
    wrapper = self._variables()
    unwrap(Variable, wrapper)

Binding_LinearConstraint.variables = _variables
Binding_QuadraticConstraint.variables = _variables
