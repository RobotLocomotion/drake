"""
Warning:
    Deprecated. Please used ``pydrake.common`` instead. This package will be
    removed on or after 2019-03-15.
"""

from .compatibility import *
from .containers import *
# - `cpp_const` does not offer public Drake symbols.
# - `cpp_param` does not offer public Drake symbols.
# - `cpp_template` does not offer public Drake symbols.
# - `deprecation` does not offer public Drake symbols.
from .eigen_geometry import *
# N.B. Since this is generic and relatively scoped, we import the module as a
# symbol.
from . import pybind11_version
