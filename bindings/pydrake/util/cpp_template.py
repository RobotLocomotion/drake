"""Provides containers for tracking instantiations of C++ templates. """

import inspect
from types import MethodType

from pydrake.util.cpp_param import get_param_names, get_param_canonical


def _get_module_name_from_stack(frame=2):
    # Infers module name from call stack.
    return inspect.getmodule(inspect.stack()[frame][0]).__name__


def get_or_init(scope, name, template_cls, *args, **kwargs):
    """Gets an existing template from a scope if it exists; otherwise, it will
    be created and registered.

    If `scope` does not have the attribute `name`, then it will be created as
    `template_cls(name, *args, **kwargs)`.
    (`module_name=...` is also set, but should not be important).

    @param scope
        Scope that contains the template object (this may not necessarily be
        the scope of the instantiation).
    @param name
        Name of the template object.
    @param template_cls
        Class (either `TemplateBase` or a derivative).
    @param args, kwargs
        Passed to the template class's constructor.
    @returns The existing (or newly created) template object.
    """
    template = getattr(scope, name, None)
    if template is None:
        if isinstance(scope, type):
            module_name = scope.__module__
        else:
            module_name = scope.__name__
        template = template_cls(name, *args, module_name=module_name, **kwargs)
        setattr(scope, name, template)
    return template


class TemplateBase(object):
    """Provides a mechanism to map parameters (types or literals) to
    instantiations, following C++ mechanics.
    """
    def __init__(self, name, allow_default=True, module_name=None):
        """
        @param name
            Name of the template object.
        @param allow_default
            (optional) Allow a default value (None) to resolve to the
            parameters of the instantiation that was first added.
        @param module_name
            Parent module for the template object.
        """
        self.name = name
        self.param_list = []
        self._allow_default = allow_default
        self._instantiation_map = {}
        if module_name is None:
            module_name = _get_module_name_from_stack()
        self._module_name = module_name
        self._instantiation_func = None

    def __getitem__(self, param):
        """Gets concrete class associate with the given arguments.

        Can be one of the following forms:
            template[param0]
            template[param0, param1, ...]
            template[(param0, param1, ...)]
            template[[param0, param1, ...]]
            template[None]   (first instantiation, if `allow_default` is True)
        """
        return self.get_instantiation(param)[0]

    # Unique token to signify that this instantiation is deferred when using
    # `add_instantiations` or `define`. The instantiation function will not be
    # called until the specific instantiation is requested. To illustrate, the
    # following example defines a template via `@define`, and refers to
    # itself:
    #
    # @TemplateClass.define("MyTemplate", ((int,), (float,)))
    # def MyTemplate(param):
    #     print(MyTemplate)
    #     # ... Make and return a class.
    #
    # If the instantiation were not deferred and the inner function was called
    # before the decorator returned, an error would be raised since
    # `MyTemplate` is not yet defined. However, since it is deferred, this
    # should print out that `MyTemplate` is `<TemplateClass ...MyTemplate>`,
    # would only be called when the user requests something such as
    # `MyTemplate[float]`.
    class _Deferred(object):
        pass

    _deferred = _Deferred()

    def get_instantiation(self, param=None, throw_error=True):
        """Gets the instantiation for the given parameters.

        @param param
            Can be None, a single parameter, or a tuple/list of parameters.
        @returns (instantiation, param), where `param` is the resolved
        set of parameters.
        """
        param = self._param_resolve(param)
        instantiation = self._instantiation_map.get(param)
        if instantiation is TemplateBase._deferred:
            assert self._instantiation_func is not None
            instantiation = self._instantiation_func(param)
            self._add_instantiation_internal(param, instantiation)
        elif instantiation is None and throw_error:
            raise RuntimeError("Invalid instantiation: {}".format(
                self._instantiation_name(param)))
        return (instantiation, param)

    def add_instantiation(self, param, instantiation):
        """Adds a unique instantiation.

        @pre `param` must not have already been added.
        """
        # Ensure that we do not already have this tuple.
        param = get_param_canonical(self._param_resolve(param))
        if param in self._instantiation_map:
            raise RuntimeError(
                "Parameter instantiation already registered: {}".format(param))
        # Register it.
        self.param_list.append(param)
        self._add_instantiation_internal(param, instantiation)
        return param

    def _add_instantiation_internal(self, param, instantiation):
        # Adds instantiation. Permits overwriting for deferred cases.
        assert instantiation is not None
        self._instantiation_map[param] = instantiation
        if instantiation is not TemplateBase._deferred:
            self._on_add(param, instantiation)

    def add_instantiations(self, instantiation_func, param_list):
        """Adds a set of instantiations given a function and a list of
        parameter sets.

        @pre This method can only be called once.
        @param instantiation_func
            Function of the form `f(template, param)`, where `template` is the
            current template and `param` is the parameter set for the current
            instantiation.
        @param param_list
            Ordered container of parameter sets that these instantiations
            should be produced for.
        """
        assert instantiation_func is not None
        if self._instantiation_func is not None:
            raise RuntimeError(
                "`add_instsantiations` cannot be called multiple times.")
        self._instantiation_func = instantiation_func
        for param in param_list:
            self.add_instantiation(param, TemplateBase._deferred)

    def get_param_set(self, instantiation):
        """Returns all parameters for a given `instantiation`.

        @returns A set of instantiations. """
        param_list = []
        for param, check in self._instantiation_map.iteritems():
            if check == instantiation:
                param_list.append(param)
        return set(param_list)

    def is_instantiation(self, obj):
        """Determines if an object is an instantion of the given template."""
        # Use `get_instantiation` so that we can handled deferred cases.
        for param in self.param_list:
            instantiation, _ = self.get_instantiation(param)
            if instantiation is obj:
                return True
        return False

    def _param_resolve(self, param):
        # Resolves to canonical parameters, including default case.
        if param is None:
            assert self._allow_default
            assert len(self.param_list) > 0
            param = self.param_list[0]
        elif not isinstance(param, tuple):
            if not isinstance(param, list):
                # Assume scalar.
                param = (param,)
            else:
                param = tuple(param)
        return get_param_canonical(param)

    def _instantiation_name(self, param):
        names = get_param_names(self._param_resolve(param))
        return '{}[{}]'.format(self.name, ', '.join(names))

    def _full_name(self):
        return "{}.{}".format(self._module_name, self.name)

    def __str__(self):
        cls_name = type(self).__name__
        return "<{} {}>".format(cls_name, self._full_name())

    def _on_add(self, param, instantiation):
        # To be overridden by child classes.
        pass

    @classmethod
    def define(cls, name, param_list, *args, **kwargs):
        """Provides a decorator for functions that defines a template using
        `name`. The template instantiations are added using
        `add_instantiations`, where the instantiation function is the decorated
        function.

        @param name
            Name of the template. This should generally match the name of the
            object being decorated for clarity.
        @param param_list
            Ordered container of parameter sets. For more information, see
            `add_instantiations`.

        Note that the name of the inner class will not matter as it will be
        overritten with the template instantiation name.
        In the below example, ``Impl` will be renamed to
        `MyTemplate[int]` when `param=(int,)`.

        Example:

        @TemplateClass.define("MyTemplate", param_list=[(int,), (float,)])
        def MyTemplate(param):
            T, = param
            class Impl(object):
                def __init__(self):
                    self.T = T
            return Impl
        """
        template = cls(name, *args, **kwargs)

        def decorator(instantiation_func):
            template.add_instantiations(instantiation_func, param_list)
            return template

        return decorator


class TemplateClass(TemplateBase):
    """Extension of `TemplateBase` for classes. """
    def __init__(self, name, override_meta=True, module_name=None, **kwargs):
        if module_name is None:
            module_name = _get_module_name_from_stack()
        TemplateBase.__init__(self, name, module_name=module_name, **kwargs)
        self._override_meta = override_meta

    def _on_add(self, param, cls):
        # Update class name for easier debugging.
        if self._override_meta:
            cls._original_name = cls.__name__
            cls._original_qualname = getattr(cls, "__qualname__", cls.__name__)
            cls.__name__ = self._instantiation_name(param)
            # Define `__qualname__` in Python2 because that's what `pybind11`
            # uses when showing function signatures when an overload cannot be
            # found.
            # TODO(eric.cousineau): When porting to Python3 / six, try to
            # ensure this handles nesting.
            cls.__qualname__ = cls.__name__
            cls.__module__ = self._module_name

    def is_subclass_of_instantiation(self, obj):
        """Determines if `obj` is a subclass of one of the instantiations.

        @return The first instantiation of which `obj` is a subclass.
        """
        for param in self.param_list:
            instantiation, _ = self.get_instantiation(param)
            if issubclass(obj, instantiation):
                return instantiation
        return None


class TemplateFunction(TemplateBase):
    """Extension of `TemplateBase` for functions. """
    pass


class TemplateMethod(TemplateBase):
    """Extension of `TemplateBase` for class methods. """
    def __init__(self, name, cls, module_name=None, **kwargs):
        if module_name is None:
            module_name = _get_module_name_from_stack()
        TemplateBase.__init__(self, name, module_name=module_name, **kwargs)
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
        def __init__(self, template, obj):
            self._tpl = template
            self._obj = obj

        def __getitem__(self, param):
            unbound = self._tpl[param]
            bound = MethodType(unbound, self._obj, self._tpl._cls)
            return bound

        def __str__(self):
            return '<bound TemplateMethod {} of {}>'.format(
                self._tpl._full_name(), self._obj)
