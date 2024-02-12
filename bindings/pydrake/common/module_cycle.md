
The lowest-level modules in pydrake have a dependency cycle: classes (or
functions) from one module refer to classes (or functions) from other modules
such that there is no ordering of module loading that ensures the referenced
module is loaded before the module that uses needs its classes (or functions).

The following modules are part of the dependency cycle:
- pydrake.autodiffutils
- pydrake.common
- pydrake.common.cpp_template
- pydrake.common.deprecation
- pydrake.common.eigen_geometry
- pydrake.common.schema
- pydrake.common.value
- pydrake.math
- pydrake.symbolic

(There are probably more submodules in python.common that are part of the cycle;
the list is not necessarily complete.)

Therefore, we must merge all of those *logical* modules into a single *physical*
module (one shared library) so the library setup code can incrementally define
symbols one at a time (bouncing between modules) in an order that satisfies all
requirements. That module is `pydrake.common`.

When the user does an `import pydrake`, that `__init__.py` file does `import
pydrake.common` and in that C++ code (our `module_py.cc` code), all of the
necessary modules (pydrake.math, etc) are defined and initialized.
