# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

from pydrake.common.deprecation import deprecated_callable as _deprecated


@_deprecated(
    "pydrake.solvers.augmented_lagrangian.NonsmoothAugmentedLagrangian has "
    "been renamed to "
    "pydrake.solvers.augmented_lagrangian.AugmentedLagrangianNonsmooth.",
    date="2022-07-01")
def NonsmoothAugmentedLagrangian(*args, **kwargs):
    from pydrake.solvers.augmented_lagrangian import \
        AugmentedLagrangianNonsmooth as real
    return real(*args, **kwargs)
