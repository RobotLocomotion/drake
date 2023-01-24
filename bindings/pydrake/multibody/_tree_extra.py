from pydrake.common import _MangledName


def __getattr__(name):
    """Rewrites requests for Foo[bar] into their mangled form, for backwards
    compatibility with unpickling.
    """
    return _MangledName.module_getattr(
        module_name=__name__, module_globals=globals(), name=name)
