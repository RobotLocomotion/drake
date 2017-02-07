from __future__ import absolute_import

import numpy as np
from ._pydrake_mathematicalprogram import *
from ..wrapperutils import wrap, unwrap


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
        if x.ndim == 1:
            wrapper_type = VectorXDecisionVariable
        elif x.ndim == 2:
            wrapper_type = MatrixXDecisionVariable
        else:
            raise TypeError("GetSolution(x) requires that x be a scalar or a 1- or 2- dimensional numpy array")
        return self._GetSolution(wrap(wrapper_type, x))
    else:
        return self._GetSolution(x)

MathematicalProgram.GetSolution = _GetSolution


def _variables(self):
    wrapper = self._variables()
    return unwrap(Variable, wrapper)

Binding_LinearConstraint.variables = _variables
Binding_QuadraticConstraint.variables = _variables
