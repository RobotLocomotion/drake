#!/usr/bin/env python

import inspect
from types import MethodType

_PARAM_DEFAULT = 'first_registered'


# Pending #7732.
def get_type_names(param):
    return [item.__name__ for item in param]


def get_types_canonical(param):
    return tuple(param)


def _get_module_name_from_stack(frame=2):
    # Infers module name from call stack.
    return inspect.getmodule(inspect.stack()[frame][0]).__name__


def get_or_init(scope, name, template_cls, *args, **kwargs):
    """Gets an existing template from a scope if it exists; otherwise, it will
    be created and registered. """
    tpl = getattr(scope, name, None)
    if tpl is None:
        if isinstance(scope, type):
            module_name = scope.__module__
        else:
            module_name = scope.__name__
        tpl = template_cls(name, *args, module_name=module_name, **kwargs)
        setattr(scope, name, tpl)
    return tpl


class Template(object):
    """Provides a mechanism to map parameters (types or literals) to
    instantiations, following C++ mechanics. """
    def __init__(self, name, param_default=_PARAM_DEFAULT, module_name=None):
        self.name = name
        if param_default and param_default != _PARAM_DEFAULT:
            param_default = get_types_canonical(param_default)
        self.param_list = []
        self._param_default = param_default
        self._instantiation_map = {}
        if module_name is None:
            module_name = _get_module_name_from_stack()
        self._module_name = module_name

    def __getitem__(self, param):
        """Gets concrete class associate with the given arguments.

        Can be one of the following forms:
            tpl[param0]
            tpl[param0, param1, ...]
            tpl[(param0, param1, ...)]
            tpl[[param0, param1, ...]]
            tpl[None]   (default instantiation)
        """
        return self.get_instantiation(param)[0]

    def get_instantiation(self, param=None, throw_error=True):
        """Gets the instantiation for the given parameters.

        @param param
            Can be None, a single parameter, or a tuple/list of parameters.
        @returns (instantiation, param), where `param` is the resolved
        set of parameters.
        """
        param = self._param_resolve(param)
        instantiation = self._instantiation_map.get(param)
        if instantiation is None and throw_error:
            raise RuntimeError("Invalid instantiation: {}".format(
                self._instantiation_name(param)))
        return (instantiation, param)

    def add_instantiation(self, param, instantiation):
        """Adds a unique instantiation. """
        assert instantiation is not None
        # Ensure that we do not already have this tuple.
        param = get_types_canonical(self._param_resolve(param))
        if param in self._instantiation_map:
            raise RuntimeError(
                "Parameter instantiation already registered: {}".format(param))
        # Register it.
        self.param_list.append(param)
        self._instantiation_map[param] = instantiation
        if self._param_default == _PARAM_DEFAULT:
            self._param_default = param
        self._on_add(param, instantiation)
        return param

    def add_instantiations(
            self, instantiation_func, param_list=None):
        """Adds a set of instantiations given a function of the form
        `instantiation_func(param)`, for a given set of parmeters
        `param_list`. """
        assert param_list is not None
        for param in param_list:
            self.add_instantiation(param, instantiation_func(param))

    def get_param_set(self, instantiation):
        """Returns all parameters for a given `instantiation`.

        @returns A set of instantiations. """
        param_list = []
        for param, check in self._instantiation_map.iteritems():
            if check == instantiation:
                param_list.append(param)
        return set(param_list)

    def _param_resolve(self, param):
        # Resolve from default argument to canonical parameters.
        if param is None:
            assert self._param_default is not None
            param = self._param_default
        elif not isinstance(param, tuple):
            if not isinstance(param, list):
                # Assume scalar.
                param = (param,)
            else:
                param = tuple(param)
        return get_types_canonical(param)

    def _instantiation_name(self, param):
        names = get_type_names(self._param_resolve(param))
        return '{}[{}]'.format(self.name, ', '.join(names))

    def _full_name(self):
        return "{}.{}".format(self._module_name, self.name)

    def __str__(self):
        cls_name = type(self).__name__
        return "<{} {}>".format(cls_name, self._full_name())

    def _on_add(self, param, instantiation):
        # To be overrode by child classes.
        pass


class TemplateClass(Template):
    """Extension of `Template` for classes. """
    def __init__(self, name, override_name=True, **kwargs):
        Template.__init__(self, name, **kwargs)
        self._override_name = override_name

    def _on_add(self, param, cls):
        # Update class name for easier debugging.
        if self._override_name:
            cls.__name__ = self._instantiation_name(param)


class TemplateFunction(Template):
    """Extension of `Template` for functions. """
    pass


class TemplateMethod(TemplateFunction):
    """Extension of `Template` for class methods. """
    def __init__(self, name, cls, **kwargs):
        TemplateFunction.__init__(self, name, **kwargs)
        self._cls = cls

    def __get__(self, obj, objtype):
        """Provides descriptor accessor. """
        if obj is None:
            return self
        else:
            return TemplateMethod._Bound(self, obj)

    def __set__(self, obj, value):
        raise RuntimeError("Read-only property")

    def __str__(self):
        return '<unbound TemplateMethod {}>'.format(self._full_name())

    def _full_name(self):
        return '{}.{}'.format(self._cls.__name__, self.name)

    class _Bound(object):
        def __init__(self, tpl, obj):
            self._tpl = tpl
            self._obj = obj

        def __getitem__(self, param):
            unbound = self._tpl[param]
            bound = MethodType(unbound, self._obj, self._tpl._cls)
            return bound

        def __str__(self):
            return '<bound TemplateMethod {} of {}>'.format(
                self._tpl._full_name(), self._obj)


def is_instantiation_of(obj, tpl):
    """Determines of an object is registered as an instantiation. """
    return obj in tpl._instantiation_map.values()
