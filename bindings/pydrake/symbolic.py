from __future__ import absolute_import, division, print_function

from ._pydrake_symbolic import *

# Explicitly import private symbols
from ._pydrake_symbolic import __logical_and, __logical_or


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return reduce(__logical_or, formulas)
