"""Provides utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import SystemScalarConverter
from pydrake.util.cpp_template import is_instantiation_of


def _get_conversion_pairs(param_list):
    # Intersect.
    T_compat = []
    for T, in param_list:
        if T in SystemScalarConverter.SupportedScalars:
            T_compat.append(T)
    # Outer join without duplicates.
    param_pairs = []
    for T in T_compat:
        for U in T_compat:
            if T != U:
                param_pairs.append((T, U))
    return param_pairs


class ScalarHelper(object):
    """A helper to handle dispatching constructors and help creating
    converters."""

    # Global cache.
    _converter_cache = {}

    def __init__(self, template):
        self._template = template

    def check_if_copying(self, *args, **kwargs):
        """Checks if a function signature implies a copy constructor.

        Determines if constructor call has one parameter which is an
        instantiation of the current template, indicating that this is a System
        scalar type conversion copy constructor.

        For example, if you have template `MyClass`, and you are defining your
        instantiations, you should do the following:

            @TemplateClass.define("MyClass", param_list=LeafSystem_.param_list)
            def MyClass(MyClass, param):
                T, = param
                LeafSystem = LeafSystem_[T]
                scalar_helper = ScalarHelper(MyClass)

                class MyClassInstantiation(LeafSystem):
                    def __init__(self, *args, **kwargs):
                        # Check if this is a copy constructor.
                        other = scalar_helper.check_if_copying(*args, **kwargs)
                        if other:
                            # Implement copy constructor.
                        else:
                            # Implement normal constructor.
                            # Note: It may be best to redirect to another
                            # function, to make it easier to use named
                            parameters.

                return MyClassInstantiation

        @return The first argument (other system) if it is, None otherwise.
        """
        if len(args) == 1 and len(kwargs) == 0:
            if is_instantiation_of(type(args[0]), self._template):
                return args[0]
        return None

    def make_converter(self, param_pairs=None):
        """Creates system scalar converter for the template class.

        Conversion is implemented using the copy constructor of the
        class instantiations.
        For example, if you have template `Class`, and you wish to convert from
        `Class[U]` to `Class[T]`, you must ensure the copy constructed portion
        of the following code works:

            system_U = Class[U](...)  # Normally constructed.
            system_T = Class[T](system_U)  # Copy constructed.

        See `check_if_copy_constructor` for how to implement this.

        @param template TemplateClass instance.
        @param param_pairs List of pairs, (T, U), defining a conversion from a
            scalar type of U to T.
            If None, this will use all possible pairs that the Python bindings
            of `SystemScalarConverter` support.
        @note This copies values from a global cache when possible.
        """
        if param_pairs is None:
            param_pairs = _get_conversion_pairs(self._template.param_list)

        cache_key = (self._template, tuple(param_pairs))
        cache_entry = self._converter_cache.get(cache_key)
        if cache_entry:
            return copy.copy(cache_entry)

        # Generate and register each conversion.
        converter = SystemScalarConverter()

        # Define capture to ensure the current values are bound, and do not
        # change through iteration.
        def add_captured(param):
            T, U = param

            def conversion(system):
                assert isinstance(system, self._template[U])
                return self._template[T](system)

            converter.Add[T, U](conversion)

        map(add_captured, param_pairs)

        self._converter_cache[cache_key] = copy.copy(converter)
        return converter
