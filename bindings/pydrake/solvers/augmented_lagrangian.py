"""Shim module that provides vestigial names for pydrake.solvers.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.solvers directly.

This module will be deprecated at some point in the future.
"""

from pydrake.common.deprecation import deprecated_callable as _deprecated

from pydrake.solvers import (
    AugmentedLagrangianNonsmooth,
    AugmentedLagrangianSmooth,
)


@_deprecated(
    "pydrake.solvers.augmented_lagrangian.NonsmoothAugmentedLagrangian has "
    "been renamed to "
    "pydrake.solvers.augmented_lagrangian.AugmentedLagrangianNonsmooth.",
    date="2022-07-01")
def NonsmoothAugmentedLagrangian(*args, **kwargs):
    from pydrake.solvers import AugmentedLagrangianNonsmooth as real
    return real(*args, **kwargs)
