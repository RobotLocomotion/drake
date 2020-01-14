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

Note:
    Import order matters! If there is a name conflict, the last one imported
    wins.

To see example usages, please see `doc/python_bindings.rst`.
"""


# Legacy symbols.
from .attic.all import *

# Normal symbols.
from . import getDrakePath
from .autodiffutils import *
from .forwarddiff import *
from .lcm import *
from .math import *
from .perception import *
from .polynomial import *
from .symbolic import *
from .trajectories import *

# Submodules.
from .common.all import *
from .geometry.all import *
# - `examples` does not offer public Drake library symbols.
from .manipulation.all import *
from .multibody.all import *
from .solvers.all import *
from .systems.all import *
# - `third_party` does not offer public Drake library symbols.
from .visualization.all import *
