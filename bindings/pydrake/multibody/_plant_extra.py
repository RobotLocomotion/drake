import warnings

from pydrake.common import _MangledName
from pydrake.common.deprecation import DrakeDeprecationWarning


def __getattr__(name):
    # Provide a deprecated alias for BaseBodyJointType, which moved to
    # pydrake.multibody.tree as of 2026-09-01.
    if name == "BaseBodyJointType":
        warnings.warn(
            "Please use ``pydrake.multibody.tree.BaseBodyJointType`` instead"
            " of ``pydrake.multibody.plant.BaseBodyJointType``."
            " The deprecated code will be removed from Drake on or after"
            " 2026-09-01.",
            category=DrakeDeprecationWarning,
            stacklevel=2,
        )
        from pydrake.multibody.tree import BaseBodyJointType

        return BaseBodyJointType

    """Rewrites requests for Foo[bar] into their mangled form, for backwards
    compatibility with unpickling.
    """
    return _MangledName.module_getattr(
        module_name=__name__, module_globals=globals(), name=name
    )
