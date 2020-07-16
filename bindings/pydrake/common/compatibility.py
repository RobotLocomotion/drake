"""Contains routines to monkey patch or provide alternatives to upstream code
which may need changes for compatibility.
"""

import numpy as np


_patches = {
    'numpy_formatters': {
        'applied': False,
        'required': np.lib.NumpyVersion(np.__version__) < '1.13.0',
    },
}


def _defer_callable_type(cls):
    # Makes a type, which is meant to purely callable, and defers its
    # construction until it needs to be called.

    class Deferred:
        def __init__(self, *args, **kwargs):
            self._args = args
            self._kwargs = kwargs
            self._obj = None

        def _get_obj(self):
            if self._obj is None:
                self._obj = cls(*self._args, **self._kwargs)
            return self._obj

        def __call__(self, *args, **kwargs):
            return self._get_obj().__call__(*args, **kwargs)

    return Deferred


def maybe_patch_numpy_formatters():
    """Required to permit printing of symbolic array types in NumPy < 1.13.0.

    See #8729 for more information.
    """
    # Provides version-dependent monkey-patch which effectively achieves a
    # portion of https://github.com/numpy/numpy/pull/8963
    patch = _patches['numpy_formatters']
    if not patch['required']:
        return
    if patch['applied']:
        return
    module = np.core.arrayprint
    defer_callable_types = [
        'IntegerFormat',
        'FloatFormat',
    ]
    for name in defer_callable_types:
        original = getattr(module, name)
        deferred = _defer_callable_type(original)
        setattr(module, name, deferred)
    patch['applied'] = True
