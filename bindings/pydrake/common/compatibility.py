"""Contains routines to monkey patch or provide alternatives to upstream code
which may need changes for compatibility, or check versions.
"""

import numpy as np
from numpy.lib import NumpyVersion

from pydrake.common.deprecation import _warn_deprecated


def check_required_numpy_version(_actual=np.version.version):
    """Fails fast if a minimum version of NumPy is not present.

    pydrake requires NumPy >= 0.15.0 namely for the following patches:
        https://github.com/numpy/numpy/pull/10898
        https://github.com/numpy/numpy/pull/11076

    If Drake uses user-dtypes in lieu of `dtype=object`, then anything that
    refers to `autodiff` or `symbolic` (e.g. all of `systems`,
    `multibody`, etc.) could potentially have "random" segfaults.
    """
    actual = NumpyVersion(_actual)
    minimum = NumpyVersion('1.15.0')
    if actual < minimum:
        raise RuntimeError(
            "pydrake requires numpy >= {}, but only {} is present".format(
                minimum.vstring, actual.vstring))


def maybe_patch_numpy_formatters():
    """Deprecated functionality."""
    _warn_deprecated(
        "`maybe_patch_numpy_formatters` is no longer needed, and will "
        "be removed after 2019/01", stacklevel=3)
