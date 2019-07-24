# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import warnings


_DEPRECATION_MESSAGE = (
    "Use primitives.RandomSource(RandomDistribution.kFoo, ...) instead of " +
    "primitives.FooRandomSource. This class will be removed on 2019-10-01.")


def UniformRandomSource(num_outputs, sampling_interval_sec):
    """Deprecated constructor that desugars to
    primitives.RandomSource(RandomDistribution.kUniform, **args, **kwargs).
    This constructor will be removed on 2019-10-01."""
    from pydrake.common import RandomDistribution
    from pydrake.common.deprecation import DrakeDeprecationWarning
    from pydrake.systems.primitives import RandomSource
    warnings.warn(_DEPRECATION_MESSAGE, category=DrakeDeprecationWarning,
                  stacklevel=2)
    return RandomSource(
            distribution=RandomDistribution.kUniform,
            num_outputs=num_outputs,
            sampling_interval_sec=sampling_interval_sec)


def GaussianRandomSource(num_outputs, sampling_interval_sec):
    """Deprecated constructor that desugars to
    primitives.RandomSource(RandomDistribution.kGaussian, **args, **kwargs).
    This constructor will be removed on 2019-10-01."""
    from pydrake.common import RandomDistribution
    from pydrake.common.deprecation import DrakeDeprecationWarning
    from pydrake.systems.primitives import RandomSource
    warnings.warn(_DEPRECATION_MESSAGE, category=DrakeDeprecationWarning,
                  stacklevel=2)
    return RandomSource(
            distribution=RandomDistribution.kGaussian,
            num_outputs=num_outputs,
            sampling_interval_sec=sampling_interval_sec)


def ExponentialRandomSource(num_outputs, sampling_interval_sec):
    """Deprecated constructor that desugars to
    primitives.RandomSource(RandomDistribution.kExponential, **args, **kwargs).
    This constructor will be removed on 2019-10-01."""
    from pydrake.common import RandomDistribution
    from pydrake.common.deprecation import DrakeDeprecationWarning
    from pydrake.systems.primitives import RandomSource
    warnings.warn(_DEPRECATION_MESSAGE, category=DrakeDeprecationWarning,
                  stacklevel=2)
    return RandomSource(
            distribution=RandomDistribution.kExponential,
            num_outputs=num_outputs,
            sampling_interval_sec=sampling_interval_sec)
