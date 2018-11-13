from pydrake.common.deprecation import ModuleShim, deprecated

value = 1


def _handler(name):
    if name == "sub_module":
        return "This would be a shim"
    else:
        raise AttributeError()


__all__ = ["value", "sub_module"]
ModuleShim._install(__name__, _handler)


class ExampleClass(object):
    doc_method = "Method Doc"
    doc_prop = "Prop Doc"
    message_method = "`deprecated_method` is deprecated"
    message_prop = "`deprecated_prop` is also deprecated"

    # Deprecate method.
    @deprecated(message_method)
    def deprecated_method(self):
        """Method Doc"""
        return 1

    # Deprecate public property.
    _deprecated_prop = property(lambda self: 2, doc=doc_prop)
    deprecated_prop = deprecated(message_prop)(_deprecated_prop)
