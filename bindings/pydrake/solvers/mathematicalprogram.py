"""
Bindings for MathematicalProgram

If you are formulating constraints using symbolic formulas, please review the
top-level documentation for :py:mod:`pydrake.math`.
"""

# This module only exists to handle deprecation; CPython C extensions cannot
# easily be swapped during construction (e.g. if calling `ModuleShim._install`
# in `_{base}_extra.py`), so we must have a forwarding module.

# The above docstring is copied from the `*_py.cc` file because (for whatever
# reason), Sphinx seems to pick up the source version (as written above), not
# the variable version (as would be assigned to `__doc__`).

from pydrake import _import_cc_module_vars
import pydrake.solvers._mathematicalprogram as _cc
from pydrake.common.deprecation import ModuleShim, _warn_deprecated

_import_cc_module_vars(_cc, __name__, include_private=True)


def _deprecation_handler(name):
    binding_prefix = "Binding_"
    if name.startswith(binding_prefix):
        cls_name = name[len(binding_prefix):]
        new_name = f"Binding[{cls_name}]"
        _warn_deprecated(
            f"{name} is deprecated; please use {new_name} instead.",
            date="2021-11-01",
        )
        cls = globals()[cls_name]
        return Binding[cls]
    else:
        raise AttributeError()


ModuleShim._install(__name__, _deprecation_handler, auto_all=True)
