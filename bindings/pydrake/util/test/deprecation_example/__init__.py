from pydrake.util.deprecation import ModuleShim, deprecated

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


class ExampleClass(object):
    doc_1 = "Doc 1"
    doc_2 = "Doc 2"
    message_1 = "`deprecated_1` is deprecated"
    message_2 = "`deprecated_2` is also deprecated"

    # Deprecate method.
    @deprecated(message_1)
    def deprecated_1(self):
        """Doc 1"""
        return 1

    # Deprecate public property.
    _deprecated_2 = property(lambda self: 2, doc=doc_2)
    deprecated_2 = deprecated(message_2)(_deprecated_2)
