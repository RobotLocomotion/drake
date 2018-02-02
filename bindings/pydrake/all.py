"""
Provides a roll-up of all user-visible symbols in `pydrake`.

Things to note:
*   The `.all` modules in `pydrake` are intended as convenient end-user
shortcut for interactive or tutorial use.
*   Code within pydrake itself should not use the all shortcut, but rather
import only exactly what is needed.
*   The downside of importing an `.all` module is (a) pulling in additional
dependencies and (b) the potential to lose a symbol if there is a conflict
(e.g. something like `pydrake.multibody.shapes.Element` vs
`pydrake.multibody.collision.Element` (which does not exist yet)).

N.B. Import order matters! If there is a name conflict, the last one imported
wins.

To see example usages, please see `doc/python_bindings.rst`.

"""

from __future__ import absolute_import

# This module.
from . import getDrakePath
from .autodiffutils import *
from .common import *
from .parsers import *
from .rbtree import *
from .symbolic import *

# Submodules.
# - Do not inclue `examples`.
from .multibody.all import *
from .solvers.all import *
from .systems.all import *
# - Do not include `third_party`.
# - Do not include `util`.
