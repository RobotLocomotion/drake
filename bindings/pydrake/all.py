"""Provides a roll-up of all user-visible modules and symbols in `pydrake`.

Things to note:

* The `.all` modules in `pydrake` are intended as convenient end-user shortcut
  for interactive or tutorial use.
* Code within pydrake itself should not use the all shortcut, but rather
  import only exactly what is needed.
* The downside of importing an `.all` module is (a) pulling in additional
  dependencies, (b) the potential to lose a symbol if there is a conflict
  (e.g. something like `pydrake.multibody.shapes.Element` vs
  `pydrake.multibody.collision.Element` (which does not exist yet)), and
  (c) deprecated symbols could get removed without warning from `all` modules.

Note:
    Import order matters! If there is a name conflict, the last one imported
    wins.

To see example usages, please see `doc/python_bindings.rst`.
"""

from __future__ import absolute_import
import warnings

# Deprecated symbols.
with warnings.catch_warnings():
    warnings.simplefilter("ignore", DeprecationWarning)
    from .util.all import *

# Legacy symbols.
from .attic.all import *

# Normal symbols.
from . import getDrakePath
from .autodiffutils import *
from .automotive import *
from .forwarddiff import *
from .geometry import *
from .lcm import *
from .math import *
from .perception import *
from .symbolic import *
from .trajectories import *

# Submodules.
from .common.all import *
# - `examples` does not offer public Drake library symbols.
from .maliput.all import *
from .manipulation.all import *
from .multibody.all import *
from .solvers.all import *
from .systems.all import *
# - `third_party` does not offer public Drake library symbols.
from .visualization.all import *
