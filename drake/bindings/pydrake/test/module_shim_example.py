from pydrake.util import ModuleShim

value = 1


def _handler(name, import_type):
    if name == "import_type":
        return import_type
    else:
        raise AttributeError()


__all__ = ["value", "import_type"]
ModuleShim.install(__name__, _handler)
