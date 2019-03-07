import warnings

# Deprecated symbols.
with warnings.catch_warnings():
    warnings.simplefilter("ignore", DeprecationWarning)
    # - Attic (#9314)
    from .collision import *
    from .joints import *
    from .parsers import *
    from .rigid_body_plant import *
    from .rigid_body_tree import *
    from .rigid_body import *
    from .shapes import *
    # - Shuffle (#9314. #9366)
    from .multibody_tree.all import *  # noqa

# Normal symbols.
from .inverse_kinematics import *  # noqa
from .math import *  # noqa
from .parsing import *  # noqa
from .plant import *  # noqa
from .tree import *  # noqa

# Submodules.
from .benchmarks.all import *  # noqa
