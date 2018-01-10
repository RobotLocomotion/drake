from pydrake.util.module_shim import ModuleShim

value = 1


def _handler(name, import_type):
    if name == "import_type":
        return import_type
    elif name == "sub_module":
        return "This would be a shim"
    else:
        raise AttributeError()


__all__ = ["value", "import_type", "sub_module"]
ModuleShim.install(__name__, _handler)
