# `pydrake` Deprecation Example

This package is used by `../deprecation_utility_test.py` and shows the
following:

* How to use `pydrake.common.deprecation` in a module, leveraging `ModuleShim`
    * `__init__.py` shows using `ModuleShim`
* How to use `drake/bindings/pydrake/common/deprecation_pybind.h` to deprecate
bound code and/or bind deprecated code:
    * `example_class.h` shows a class with example deprecations
    * `cc_module_py.cc` shows binding this class with its deprecations,
    including how the docstrings are affected.
