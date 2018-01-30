"""
Provides a roll-up of all user-visible symbols in `pydrake`.

Please *only* use `all` modules when the convenience is worth it, e.g.
for introductory examples, or when tinkering in a REPL interface.
If writing production code, *please* use the actual modules.

N.B. Import order matters! If there is a name conflict, the last one imported
wins.
"""

from __future__ import absolute_import

# This module.
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
