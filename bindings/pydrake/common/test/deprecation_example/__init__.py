from pydrake.common.deprecation import (
    ModuleShim,
    deprecated,
    deprecated_callable,
)

value = 1


def _handler(name):
    if name == "sub_module":
        return "This would be a shim"
    else:
        raise AttributeError()


__all__ = ["value", "sub_module"]
ModuleShim._install(__name__, _handler)


class ExampleClass:
    doc_method = "Method Doc"
    doc_prop = "Prop Doc"
    message_method = "`deprecated_method` is deprecated"
    message_prop = "`deprecated_prop` is also deprecated"

    # Deprecate method.
    @deprecated(message_method, date="2038-01-19")
    def deprecated_method(self):
        """Method Doc"""
        return 1

    # Deprecate public property.
    _deprecated_prop = property(lambda self: 2, doc=doc_prop)
    deprecated_prop = deprecated(message_prop, date="2038-01-19")(
        _deprecated_prop)


message_func = "`deprecated_func` is deprecated"


@deprecated_callable(message_func, date="2038-01-19")
def deprecated_func(x):
    return 2 * x
