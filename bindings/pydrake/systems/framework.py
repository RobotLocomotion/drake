"""
This module only exists to handle deprecation.

Actual docstring will be overridden programmatically.
"""
from pydrake import _import_cc_module_vars
import pydrake.common.deprecation as _deprecation
import pydrake.common.value as _value
from pydrake.systems import _framework as _cc

__doc__ = _cc.__doc__
__all__ = _import_cc_module_vars(_cc, __name__)


def _getattr(name):
    if name in ["AbstractValue", "Value"]:
        old_symbol = f"{__name__}.{name}"
        new_symbol = f"{_value.__name__}.{name}"
        _deprecation._warn_deprecated(
            f"{old_symbol} has moved to {new_symbol}. Please use it instead. "
            f"This will be removed on or after 2020-08-01.")
        return getattr(_value, name)
    else:
        raise AttributeError()


_deprecation.ModuleShim._install(__name__, _getattr)
