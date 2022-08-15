"""Shim module that provides vestigial names for pydrake.solvers.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.solvers directly.

This module will be deprecated at some point in the future.
"""

from pydrake.solvers import (
    OsqpSolver,
    OsqpSolverDetails,
)
