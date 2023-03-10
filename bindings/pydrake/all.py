"""Provides a roll-up of all user-visible modules and symbols in `pydrake`.

Things to note:

* The `.all` modules in `pydrake` are intended as convenient end-user shortcut
  for interactive or tutorial use. Please only use it for prototyping, but do
  not use it in production code.
* Code within pydrake itself should not use the all shortcut, but rather
  import only exactly what is needed.
* The downsides of importing an `.all` module are (a) pulling in additional
  dependencies, (b) the potential to lose a symbol if there is a conflict
  (e.g. something like `pydrake.multibody.shapes.Element` vs
  `pydrake.multibody.collision.Element` (which does not exist yet)), and
  (c) deprecated symbols could get removed without warning from `all` modules.
* Deprecated modules will *not* be incorporated in `all` modules, because
  otherwise, `all` would emit noisy deprecation warnings, or if they are
  suppressed, subseqeuent imports of those deprecated modules will not trigger
  warnings.

To see example usages, please see `doc/python_bindings.rst`.

Note:
    Import order matters! If there is a name conflict, the last one imported
    wins. See "Preferred Ordering" section below.

**Preferred Ordering**: In service of maintaining convenient spellings, we have
a "preferred ordering" section in this top-level module. Please note that this
ordering is subject to change without notice. If you desire stability, please
**do not use** ``pydrake.all``. If you have a preferred ordering, please submit
a PR that will be subject to discussion and review. If you wish to debug
collisions, please run ``bazel run //bindings/pydrake:print_symbol_collision``
from the Drake source tree.
"""

import inspect as __inspect

# Normal symbols.
from . import getDrakePath
from .autodiffutils import *
from .forwarddiff import *
from .lcm import *
from .manipulation import *
from .math import *
from .perception import *
from .planning import *
from .polynomial import *
from .solvers import *
from .symbolic import *
from .trajectories import *

# Submodules.
from .common.all import *
from .geometry.all import *
# - `examples` does not offer public Drake library symbols.
from .multibody.all import *
from .systems.all import *
from .visualization import *

# Preferred Ordering.
# Please note this will *re*import some modules.
# To view what collisions may occur, please run:
# - Ensure .math imports win over less capable .symbolic or .autodiffutils
# overloads.
from .math import *
# - Ensure symbolic.Polynomial wins (#18353).
from .symbolic import Polynomial
