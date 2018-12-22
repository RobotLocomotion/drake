#pragma once

#include "pybind11/pybind11.h"

// N.B. Avoid including other headers, such as `pybind11/eigen.sh` or
// `pybind11/functional.sh`, such that modules can opt-in to (and pay the cost
// for) these binding capabilities.

namespace drake {
namespace pydrake {

/** @defgroup python_bindings Python Bindings
@ingroup technical_notes
@brief Details on implementing python bindings for the C++ code.

# Overview

Drake uses [pybind11](http://pybind11.readthedocs.io/en/stable/) for binding
its C++ API to Python.

At present, a fork of `pybind11` is used which permits bindings matrices with
`dtype=object`, passing `unique_ptr` objects, and prevents aliasing for Python
classes derived from `pybind11` classes.

## Module Organization

The structure of the bindings generally follow the *directory structure*, not
the namespace structure. As an example, if in C++  you do:

    #include <drake/multibody/plant/{header}.h>
    using drake::multibody::{symbol};

then in Python you would do:

    from pydrake.multibody.plant import {symbol}

Some (but not all) exceptions:

- Some of `drake/common` is incorporated into `pydrake.util`. (This will be
remedied in the future.)
- `drake/multibody/rigid_body_tree.h` is actually contained in the module
`pydrake.multibody.rigid_body_tree`.
- `drake/solvers/mathematical_program.h` is actually contained in the module
`pydrake.solvers.mathematicalprogram`.

## `pybind11` Tips

### Python Types

Throughout the Drake code, Python types provided by `pybind11` are used, such
as `py::handle`, `py::object`, `py::module`, `py::str`, `py::list`, etc.
For an overview, see the
[pybind11 reference](http://pybind11.readthedocs.io/en/stable/reference.html).

All of these are effectively thin wrappers around `PyObject*`, and thus can be
cheaply copied.

Mutating the referred-to object also does not require passing by reference, so
you can always pass the object by value for functions, but you should document
your method if it mutates the object in a non-obvious fashion.

### Python Type Conversions

You can implicit convert between `py::object` and its derived classes (such
as `py::list`, `py::class_`, etc.), assuming the actual Python types agree.
You may also implicitly convert from `py::object` (and its derived classes) to
`py::handle`.

If you wish to convert a `py::handle` (or `PyObject*`) to `py::object` or a
derived class, you should use
[`py::reinterpret_borrow<>`](http://pybind11.readthedocs.io/en/stable/reference.html#_CPPv218reinterpret_borrow6handle).

# Conventions

## API Names

Any Python bindings of C++ code will maintain C++ naming conventions, as well
as Python code that is directly related to C++ symbols (e.g. shims, wrappers,
or extensions on existing bound classes).

All other Python code be Pythonic and use PEP 8 naming conventions.

## Target Conventions

### Names

- `*_py`: A Python library (can be pure Python or pybind)
  - File Names: `*.py`, `*_py.cc`

- `*_pybind`: A C++ library for adding pybind-specific utilities to be consumed
  by C++.
  - File Names: `*_pybind.{h,cc}`

File names should follow form with their respective target.

### Visibility

- All Python libraries should generally be private, as `pydrake` will
  be consumed as one encapsulated target.

- All C++ `*_pybind` libraries for binding utilities should be public to aide
  downstream Bazel projects. If the API is unstable, consider making it private
  with a TODO to make public once it stabilizes.

### Bazel

Given that `libdrake.so` relies on static linking for components,
any common headers should be robust against ODR violations. This can
be normally achieved by using header-only libraries.

For upstream dependencies of these libraries, do NOT depend on the direct
targets (e.g. `//common:essential`), because this will introduce runtime ODR
violations for objects that have static storage (UID counters, etc.).

Instead, you must temporarily violate IWYU because it will be satisfied by
`drake_pybind_library`, which will incorporate `libdrake.so` and the transitive
headers.

If singletons are required (e.g. for `util/cpp_param_pybind`), consider storing
the singleton values using Python.

If you are developing bindings for a small portion of Drake and would like to
avoid rebuilding a large number of components when testing, consider editing
`//tools/install/libdrake:build_components.bzl` to reduce the number of
components being built.

## pybind Module Definitions

- Any Drake pybind module should include this header file, `pydrake_pybind.h`.
- `PYBIND_MODULE` should be used to define modules.
- Modules should be defined within the namespace `drake::pydrake`.
- The alias `namespace py = pybind11` is defined as `drake::pydrake::py`. Drake
modules should not re-define this alias at global scope.
- If a certain namespace is being bound (e.g. `drake::systems::sensors`), you
may use `using namespace drake::systems::sensors` within functions or
anonymous namespaces. Avoid `using namespace` directives otherwise.

@anchor PydrakeDoc
## Documentation

Drake uses a modified version of `mkdoc.py` from `pybind11`, where `libclang`
Python bindings are used to generate C++ docstrings accessible to the C++
binding code.

An example of incorporating docstrings:

    #include "drake/bindings/pydrake/documentation_pybind.h"

    PYBIND11_MODULE(math, m) {
      using namespace drake::math;
      constexpr auto& doc = pydrake_doc.drake.math;
      using T = double;
      py::class_<RigidTransform<T>>(m, "RigidTransform", doc.RigidTransform.doc)
          .def(py::init(), doc.ExampleClass.ctor.doc_0args)
          ...
          .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
              doc.RigidTransform.ctor.doc_1args)
          .def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
              py::arg("quaternion"), py::arg("p"),
              doc.RigidTransform.ctor.doc_2args_quaternion_p)
          ...
          .def("set_rotation", &RigidTransform<T>::set_rotation, py::arg("R"),
              doc.RigidTransform.set_rotation.doc)
      ...
    }

To view the documentation rendered in Sphinx:

    bazel run //bindings/pydrake/doc:serve_sphinx [-- --browser=false]

@note Drake's online Python documentation is generated on Ubuntu Bionic, and it
is suggested to preview documentation using this platform. Other platforms may
have slightly different generated documentation.

To browse the generated documentation strings that are available for use (or
especially, to find out the names for overloaded functions' documentation),
generate and open the docstring header:

    bazel build //bindings/pydrake:generate_pybind_documentation_header
    $EDITOR bazel-genfiles/bindings/pydrake/documentation_pybind.h

Search the comments for the symbol of interest, e.g.,
`drake::math::RigidTransform::RigidTransform<T>`, and view the include file and
line corresponding to the symbol that the docstring was pulled from.

@note This file may be large, on the order of ~100K lines; be sure to use an
efficient editor!

For more detail:

- Each docstring is stored in `documentation_pybind.h` in the nested structure
`pydrake_doc`.
- The docstring for a symbol without any overloads will be accessible via
`pydrake_doc.drake.{namespace...}.{symbol}.doc`.
- The docstring for an overloaded symbol will be `.doc_something` instead of
just `.doc`, where the `_something` suffix conveys some information about the
overload.  Browse the documentation_pybind.h (described above) for details.
Most commonly, the names will be `doc_1args`, `doc_3args`, etc.  Be sure that
the pydrake binding's signature is consistent with the docstring argument
count.
- To suppress a Doxygen comment from mkdoc, add the custom Doxygen command
`@exclude_from_pydrake_mkdoc{Explanatory text.}` to the API comment text.
(This is useful to help dismiss unbound overloads, so that mkdoc's choice of
`_something` name suffix is simpler for the remaining overloads.)

@anchor PydrakeKeepAlive
## Keep Alive Behavior

`py::keep_alive` is used heavily throughout this code. For more
information, please see [the pybind11 documentation](
http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive).

Terse notes are added to method bindings to indicate the patient
(object being kept alive by nurse) and the nurse (object keeping patient
alive). To expand on them:
- "Keep alive, ownership" implies that one argument is owned directly by
one of the other arguments (`self` is included in those arguments, for
`py::init<>` and class methods).
- "Keep alive, reference" implies a reference that is lifetime-sensitive
(something that is not necessarily owned by the other arguments).
- "Keep alive, transitive" implies a transfer of ownership of owned
objects from one container to another (e.g. transferring all `System`s
from `DiagramBuilder` to `Diagram` when calling
`DiagramBuilder.Build()`).

@anchor PydrakeOverloads
## Function Overloads

To bind function overloads, please try the following (in order):
- `py::overload_cast<Args>(func)`: See [the pybind11
documentation](http://pybind11.readthedocs.io/en/stable/classes.html#overloaded-methods).
This works about 80% of the time.
- `pydrake::overload_cast_explicit<Return, Args...>(func)`: When
`py::overload_cast` does not work (not always guaranteed to work).
- `static_cast`, as mentioned in the pybind11 documentation.
- Lambdas, e.g. `[](Args... args) -> auto&& { return func(args...); }`
(using perfect forwarding when appropriate).

## Python Subclassing of C++ Classes

In general, minimize the amount in which users may subclass C++ classes in
Python. When you do wish to do this, ensure that you use a trampoline class
in `pybind`, and ensure that the trampoline class inherits from the
`py::wrapper<>` class specific to our fork of `pybind`. This ensures that no
slicing happens with the subclassed instances.

## Convenience aliases

Some aliases are provided; prefer these to the full spellings.

`namespace py` is a shorthand alias to `pybind11` for consistency.

@see py_reference, py_reference_internal for dealing with %common ownership
     issues.

@note Downstream users should avoid `using namespace drake::pydrake`, as
this may create ambiguous aliases (especially for GCC). Instead, consider
an alias.

@anchor PydrakeBazelDebug
# Interactive Debugging with Bazel

If you are debugging a unitest, first try running the test with `--trace=user`
to see where the code is failing. This should cover most cases where you need
to debug C++ bits. Example:

    bazel run //bindings/pydrake/systems:py/lifetime_test -- --trace=user

If you need to debug futher while using Bazel, it is suggested to use
`gdbserver` for simplicity. Example:

```
    # Terminal 1 - Host process.
    cd drake
    bazel run -c dbg \
        --run_under='gdbserver localhost:9999' \
        //bindings/pydrake/systems:py/lifetime_test -- \
        --trace=user

    # Terminal 2 - Client debugger.
    cd drake
    gdb -ex "dir ${PWD}/bazel-drake" \
        -ex "target remote localhost:9999" \
        -ex "set sysroot" \
        -ex "set breakpoint pending on"
    # In the GDB terminal:
    (gdb) break drake::systems::Simulator<double>::Simulator
    (gdb) continue
```

`set sysroot` is important for using `gdbserver`, `set breakpoint pending on`
allows you set the breakpoints before loading `libdrake.so`, and `dir ...` adds
source directories. It is also suggested that you
[enable readline history](https://stackoverflow.com/a/3176802) in `~/.gdbinit`
for ease of use.

If using CLion, you can still connect to the `gdbserver` instance.

There are analogs for `lldb` / `lldbserver` but for brevity, only GDB is
covered.

*/

// TODO(eric.cousineau): If it ever stops redirecting stdin, use
// `bazel run --run_under='gdb --args python' --script_path=...`.

// Note: Doxygen apparently doesn't process comments for namespace aliases. If
// you put Doxygen comments here they will apply instead to py_reference. See
// the "Convenience aliases" section above for documentation.
namespace py = pybind11;

/// Used when returning `T& or `const T&`, as pybind's default behavior is to
/// copy lvalue references.
const auto py_reference = py::return_value_policy::reference;

/// Used when returning references to objects that are internally owned by
/// `self`. Implies both `py_reference` and `py::keep_alive<0, 1>`, which
/// implies "Keep alive, reference: `return` keeps` self` alive".
const auto py_reference_internal = py::return_value_policy::reference_internal;

// Implementation for `overload_cast_explicit`. We must use this structure so
// that we can constrain what is inferred. Otherwise, the ambiguity confuses
// the compiler.
template <typename Return, typename... Args>
struct overload_cast_impl {
  auto operator()(Return (*func)(Args...)) const { return func; }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...)) const {
    return method;
  }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...) const) const {
    return method;
  }
};

/// Provides option to provide explicit signature when
/// `py::overload_cast<Args...>` fails to infer the Return argument.
template <typename Return, typename... Args>
constexpr auto overload_cast_explicit = overload_cast_impl<Return, Args...>{};

/// Binds Pythonic `__copy__` and `__deepcopy__` for a class's copy
/// constructor.
/// @note Do not use this if the class's copy constructor does not imply a deep
/// copy.
template <typename PyClass>
void DefCopyAndDeepCopy(PyClass* ppy_class) {
  using Class = typename PyClass::type;
  PyClass& py_class = *ppy_class;
  py_class.def("__copy__", [](const Class* self) { return Class{*self}; })
      .def("__deepcopy__",
          [](const Class* self, py::dict /* memo */) { return Class{*self}; });
}

/// Executes Python code to introduce additional symbols for a given module.
/// For a module with local name `{name}`, the code executed will be
/// `_{name}_extra.py`. See #9599 for relevant background.
inline void ExecuteExtraPythonCode(py::module m) {
  py::module::import("pydrake").attr("_execute_extra_python_code")(m);
}

#if PY_MAJOR_VERSION >= 3
// The following works around pybind11 modules getting reconstructed /
// reimported in Python3. See pybind/pybind11#1559 for more details.
// Use this ONLY when necessary (e.g. when using a utility method which imports
// the module, within the module itself).
#define PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(variable)                 \
  {                                                                       \
    static py::handle variable##_original;                                \
    if (variable##_original) {                                            \
      variable = py::reinterpret_borrow<py::module>(variable##_original); \
      return;                                                             \
    } else {                                                              \
      variable##_original = variable;                                     \
    }                                                                     \
  }
#else  // PY_MAJOR_VERSION >= 3
#define PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(variable)
#endif  // PY_MAJOR_VERSION >= 3

}  // namespace pydrake
}  // namespace drake
