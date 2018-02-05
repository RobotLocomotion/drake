#pragma once

#include <pybind11/pybind11.h>

// N.B. Avoid including other headers, such as `pybind11/eigen.sh` or
// `pybind11/functional.sh`, such that modules can opt-in to (and pay the cost
// for) these binding capabilities.

namespace drake {
namespace pydrake {

/**
@page python_bindings Python Bindings

Drake uses [pybind11](http://pybind11.readthedocs.io/en/stable/) for binding
its C++ API to Python.

At present, a fork of `pybind11` is used which permits bindings matrices with
`dtype=object`, passing `unique_ptr` objects, and prevents aliasing for Python
classes derived from `pybind11` classes.

# Conventions

## Target Conventions

Target names should be of the following form:

- `*_py`: A Python library (can be pure Python or pybind)
  - File Names: `*.py`, `*_py.cc`

- `*_pybind`: A C++ library for adding pybind-specific utilities to be consumed
  by C++.
  - File Names: `*_pybind.{h,cc}`

File names should follow form with their respective target.

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
objects from one container to another (e.g. transfering all `System`s
from `DiagramBuilder` to `Diagram` when calling
`DiagramBuilder.Build()`).

# Interactive Debugging with Bazel

If you would like to interactively debug binding code (using IPython for
general Python behavior, or GDB for C++ behavior), debug C++ behavior from a
Python binary, while using Bazel, you may expose Bazel's development
environment variables by adding these lines to your Python script:

    import subprocess
    subprocess.Popen(
        "export -p | sed 's# PWD=# OLD_PWD=#g' | tee /tmp/env.sh",
        shell=True)

Run your target once from Bazel, and then source the generated `/tmp/env.sh` in
your terminal to gain access to the environment variables (e.g. `$PYTHONPATH`).

## Example with GDB

This is a brief recipe for debugging with GDB
(note the usage of subshell `(...)` to keep the variables scoped):

    (
        target=//bindings/pydrake/systems:lifetime_test
        target_bin=$(echo ${target} | sed -e 's#//##' -e 's#:#/#')
        bazel run -c dbg ${target}
        workspace=$(bazel info workspace)
        name=$(basename ${workspace})
        cd ${workspace}/bazel-${name}
        source /tmp/env.sh
        gdb --args python ${workspace}/bazel-bin/${target_bin}
    )

This allows you to use GDB from the terminal, while being able to inspect the
sources in Bazel's symlink forests.

If using CLion, consider using `gdbserver`.

*/

// TODO(eric.cousineau): Add API naming conventions (#7819).

/// @defgroup Convenience aliases
/// @{

/// Shorthand alias to `pybind` for consistency.
/// @note Downstream users should avoid `using namespace drake::pydrake`, as
/// this may create ambiguous aliases (especially for GCC). Instead, consider
/// an alias.
namespace py = pybind11;

/// Used when returning `T& or `const T&`, as pybind's default behavior is to
/// copy lvalue references.
const auto py_reference_internal =
    py::return_value_policy::reference_internal;

/// Used when returning references to objects that are internally owned by
/// `self`. Implies both `py_reference` and `py::keep_alive<0, 1>`, which
/// implies "Keep alive, reference: `return` keeps` self` alive".
const auto py_reference = py::return_value_policy::reference;

/// @}

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

// TODO(eric.cousineau): pybind11 defaults to C++-like copies when dealing
// with rvalues. We should wrap this into a drake-level binding, so that we
// can default this to `py_reference` or `py_reference_internal.`

}  // namespace pydrake
}  // namespace drake
