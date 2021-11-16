/** @file
 Doxygen-only documentation for @ref python_bindings.  */

/** @defgroup python_bindings Python Bindings
@ingroup technical_notes
@brief Details on implementing python bindings for the C++ code.

# Overview

Drake uses [pybind11](http://pybind11.readthedocs.io/en/stable/) for binding
its C++ API to Python.

At present, a fork of `pybind11` is used which permits bindings matrices with
`dtype=object`, passing `unique_ptr` objects, and prevents aliasing for Python
classes derived from `pybind11` classes.

Before delving too deep into this, please first review the user-facing
documentation about
[What's Available from Python](https://drake.mit.edu/python_bindings.html#what-s-available-from-python).

## Module Organization

<!--
TODO(eric.cousineau): Migrate the header -> module mapping to user docs.
-->

The structure of the bindings generally follow the *directory structure*, not
the namespace structure. As an example, the following code in C++:

```{.cc}
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
```

will look similar in Python, but you won't use the header file's name:

```{.py}
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
```

In general, you can find where a symbol is bound by searching for the symbol's
name in quotes underneath the directory `drake/bindings/pydrake`.

For example, you should search for `"MultibodyPlant"`, `"Parser"`, etc. To
elaborate, the binding of `Parser` is found in
`.../pydrake/multibody/parsing_py.cc`, and looks like this:

```{.cc}
using Class = Parser;
py::class_<Class>(m, "Parser", ...)
```

and binding of `MultibodyPlant` template instantiations are in
`.../pydrake/multibody/plant_py.cc` and look like this:

```{.cc}
using Class = MultibodyPlant<T>;
DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
    m, "MultibodyPlant", ...)
```

where the function containing this definition is templated on type `T` and
invoked for the scalar types mentioned in `drake/common/default_scalars.h`.

Some (but not all) exceptions to the above rules:

- `drake/common/autodiff.h` symbols live in `pydrake.autodiffutils`.
- `drake/common/symbolic.h` symbols live in `pydrake.symbolic`.
- `drake/common/value.h` symbols live in `pydrake.common.value`.
- `drake/solvers/mathematical_program.h` symbols live in
`pydrake.solvers.mathematicalprogram` (notice the Python name has no
underscore).
- `drake/systems/framework/*.h` symbols live in `pydrake.systems.framework`
(per the rule) and the bindings are ultimately linked via
`pydrake/systems/framework_py.cc`, but for compilation speed the binding
definitions themselves are split into
`./framework_py_{semantics,systems,values}.cc`.

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

## API

Any Python bindings of C++ code will maintain C++ naming conventions, as well
as Python code that is directly related to C++ symbols (e.g. shims, wrappers,
or extensions on existing bound classes).

All other Python code be Pythonic and use PEP 8 naming conventions.

For binding functions or methods, argument names should be provided that
correspond exactly to the C++ signatures using `py::arg("arg_name")`. This
permits the C++ documentation to be relevant to the Sphinx-generated
@ref PydrakeDoc "documentation", and allows for the keyword-arguments to be
used in Python.

For binding functions, methods, properties, and classes, docstrings should be
provided. These should be provided as described @ref PydrakeDoc "here".

## Testing

In general, since the Python bindings wrap tested C++ code, you do not (and
should not) repeat intricate testing logic done in C++. Instead, ensure you
exercise the Pythonic portion of the API, using kwargs when appropriate.

When testing the values of NumPy matrices, please review the documentation in
`pydrake.common.test_utilities.numpy_compare` for guidance.

## Target Conventions

### Names

- `*_py`: A Python library (can be pure Python or pybind)
  - File Names: `*.py`, `*_py.cc`
    - Each `*_py.cc` file should only define one package (a module; optionally
    with multiple submodules under it).
    - *Note*: If you need to split up a `{module}_py.cc` file for compilation
    speed and clarity, use `{module}_py_{part}.cc` for source and
    `{module}_py_{part}.h` for headers, and then include the headers into the
    original module source file. `{part}` may not necessarily be a submodule.

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

@anchor PydrakeModuleDefinitions
## pybind Module Definitions

- Modules should be defined within the drake::pydrake namespace. Please review
this namespace for available helper methods / classes.
- Any Drake pybind module should include `pydrake_pybind.h`.
- `PYBIND_MODULE` should be used to define modules.
- The alias `namespace py = pybind11` is defined as `drake::pydrake::py`. Drake
modules should not re-define this alias at global scope.
- If a certain namespace is being bound (e.g. `drake::systems::sensors`), you
may use `using namespace drake::systems::sensors` within functions or
anonymous namespaces. Avoid `using namespace` directives otherwise.
- Any symbol referenced in a module binding (even as function/method parameters)
must either be *bound* in that compilation unit (with the binding evaluated
before to the reference), or the module must import the pydrake module in which
it is bound (e.g., `py::module::import("pydrake.foo"))`). Failure to do so can
cause errors (unable to cast unregistered types from Python to C++) and can
cause the generated docstring from pybind11 to render these types by their C++
`typeid` rather than the Python type name.
- If a module depends on the *bindings* for another module, then you should do
the following:
  - Ensure that the dependent bindings module is listed in the `py_deps`
  attribute for the `drake_pybind_library()` target.
  - Inside the `_py.cc`, ensure that you tell Python to import dependency
  bindings. This is important to load the bindings at the right time (for
  documentation), and ensure the dependent bindings are executed so that users
  can import that module in isolation and still have it work. This is important
  when your type signatures use dependency bindings, or a class declares its
  inheritance, etc.
  - As an example:

        # .../my_component.h
        #include "drake/math/rigid_transform.h"
        RigidTransformd MyMethod();

        # bindings/.../BUILD.bazel
        drake_pybind_library(
            name = "my_method_py",
            ...
            py_deps = [
                ...
                "//bindings/pydrake:math_py",
            ],
            ...
        )

        # bindings/.../my_method_py.cc
        PYBIND_MODULE(my_method, m) {
          py::module::import("pydrake.math");
          m.def("MyMethod", &MyMethod, ...);
        }

@anchor PydrakeDoc
## Documentation

Drake uses a modified version of `mkdoc.py` from `pybind11`, where `libclang`
Python bindings are used to generate C++ docstrings accessible to the C++
binding code.

These docstrings are available within `constexpr struct ... pydrake_doc`
as `const char*` values . When these are not available or not suitable for
Python documentation, provide custom strings. If this custom string is long,
consider placing them in a heredoc string.

An example of incorporating docstrings from `pydrake_doc`:

```{.cc}
    #include "drake/bindings/pydrake/documentation_pybind.h"

    PYBIND11_MODULE(math, m) {
      using namespace drake::math;
      constexpr auto& doc = pydrake_doc.drake.math;
      using T = double;
      py::class_<RigidTransform<T>>(m, "RigidTransform", doc.RigidTransform.doc)
          .def(py::init(), doc.RigidTransform.ctor.doc_0args)
          ...
          .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
              doc.RigidTransform.ctor.doc_1args_R)
          .def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
              py::arg("quaternion"), py::arg("p"),
              doc.RigidTransform.ctor.doc_2args_quaternion_p)
          ...
          .def("set_rotation", &RigidTransform<T>::set_rotation, py::arg("R"),
              doc.RigidTransform.set_rotation.doc)
      ...
    }
```

An example of supplying custom strings:

```{.cc}
    constexpr char another_helper_doc[] = R"""(
    Another helper docstring. This is really long.
    And has multiple lines.
    )""";

    PYBIND11_MODULE(example, m) {
      m.def("helper", []() { return 42; }, "My helper method");
      m.def("another_helper", []() { return 10; }, another_helper_doc);
    }
```

@note Consider using scoped aliases to abbreviate both the usage of bound types
and the docstring structures. Borrowing from above:

```{.cc}
    {
      using Class = RigidTransform<T>;
      constexpr auto& cls_doc = doc.RigidTransform;
      py::class_<Class>(m, "RigidTransform", cls_doc.doc)
          .def(py::init(), cls_doc.ctor.doc_0args)
          ...
    }
```

To view the documentation rendered in Sphinx:

    bazel run //doc/pydrake:serve_sphinx [-- --browser=false]

@note Drake's online Python documentation is generated on Ubuntu Bionic, and it
is suggested to preview documentation using this platform. Other platforms may
have slightly different generated documentation.

To browse the generated documentation strings that are available for use (or
especially, to find out the names for overloaded functions' documentation),
generate and open the docstring header:

    bazel build //bindings/pydrake:documentation_pybind.h
    $EDITOR bazel-bin/bindings/pydrake/documentation_pybind.h

Search the comments for the symbol of interest, e.g.,
`drake::math::RigidTransform::RigidTransform<T>`, and view the include file and
line corresponding to the symbol that the docstring was pulled from.

@note This file may be large, on the order of ~100K lines; be sure to use an
efficient editor!

@note If you are debugging a certain file and want quicker generation and a
smaller generated file, you can hack `mkdoc.py` to focus only on your include
file of chioce. As an example, debugging `mathematical_program.h`:
```{.py}
    ...
    assert len(include_files) > 0  # Existing code.
    include_files = ["drake/solvers/mathematical_program.h"]  # HACK
```
This may break the bindings themselves, and should only be used for inspecting
the output.

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
- If two or more docstrings are the same, only one new symbol is introduced.
- To suppress a Doxygen comment from mkdoc, add the custom Doxygen command
@c \@exclude_from_pydrake_mkdoc{Explanation} to the API comment text.
This is useful to help dismiss unbound overloads, so that mkdoc's choice of
`_something` name suffix is simpler for the remaining overloads, especially if
you see the symbol `.doc_was_unable_to_choose_unambiguous_names` in the
generated documentation.
- To manually specify the `.doc_foobar` identifier name, add the line
@c \@pydrake_mkdoc_identifier{foobar} to the Doxygen comment.
- The docstring for a method that is marked as deprecated in C++ Doxygen will
be named `.doc_deprecated...` instead of just `.doc...`.

@anchor PydrakeDeprecation
## Deprecation

Decorators and utilites for deprecation in pure Python are available in
[`pydrake.common.deprecation`](https://drake.mit.edu/pydrake/pydrake.common.deprecation.html).

Deprecations for Python bindings in C++ are available in
[`drake/bindings/pydrake/common/deprecation_pybind.h`](https://drake.mit.edu/doxygen_cxx/deprecation__pybind_8h.html).

For examples of how to use the deprecations and what side effects they will
have, please see:

- [`drake/bindings/.../deprecation_example/`](https://github.com/RobotLocomotion/drake/tree/master/bindings/pydrake/common/test/deprecation_example)
- [`drake/bindings/.../deprecation_utility_test.py`](https://github.com/RobotLocomotion/drake/blob/master/bindings/pydrake/common/test/deprecation_utility_test.py)

@note All deprecations in Drake should ultimately use the
[Python `warnings` module](https://docs.python.org/3.6/library/warnings.html),
and the
[`DrakeDeprecationWarning`](https://drake.mit.edu/pydrake/pydrake.common.deprecation.html#pydrake.common.deprecation.DrakeDeprecationWarning)
class. The utilities mentioned above use them.

@anchor PydrakeKeepAlive
## Keep Alive Behavior

`py::keep_alive<Nurse, Patient>()` is used heavily throughout this code. Please
first review [the pybind11 documentation](
http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive).

`py::keep_alive` decorations should be added after all `py::arg`s are
specified. Terse comments should be added above these decorations to indicate
the relationship between the Nurse and the Patient and decode the meaning of
the Nurse and Patient integers by spelling out either the `py::arg` name (for
named arguments), `return` for index 0, or `self` (not `this`) for index 1 when
dealing with methods / members. The primary relationships:
- "Keep alive, ownership" implies that a Patient owns the Nurse (or vice
versa).
- "Keep alive, reference" implies a Patient that is referred to by the Nurse.
If there is an indirect / transitive relationship (storing a reference to
an argument's member or a transfer of ownership, as with
`DiagramBuilder.Build()`), append `(tr.)` to the relationship.

Some example comments:

```
// Keep alive, reference: `self` keeps `context` alive.
// Keep alive, ownership (tr.): `return` keeps `self` alive.
```

@anchor PydrakeReturnValuePolicy
## Return Value Policy

For more information about `pybind11` return value policies, see [the pybind11
documentation](
https://pybind11.readthedocs.io/en/stable/advanced/functions.html#return-value-policies).

`pydrake` offers the @ref drake::pydrake::py_rvp "py_rvp" alias to help with
shortened usage of `py::return_value_policy`. The most used (non-default)
policies in `pydrake` are `reference` and `reference_internal` due to the usage
of raw pointers / references in the public C++ API (rather than
`std::shared_ptr<>`).

@note While `py_rvp::reference_internal` effectively implies
`py_rvp::reference` and `py::keep_alive<0, 1>()`, we choose to only use it when
`self` is the intended patient (i.e. the bound item is a class method). For
static / free functions, we instead explicitly spell out `py_rvp::reference`
and `py::keep_alive<0, 1>()`.

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

### Mutable vs. `const` Method Overloads

C++ has the ability to distinguish `T` and `const T` for both function arguments
and class methods. However, Python does not have a native mechanism for this. It
is possible to provide a feature like this in Python (see discussion and
prototypes in [#7793](https://github.com/RobotLocomotion/drake/issues/7793));
however, its pros (similarity to C++) have not yet outweighted the cons (awkard
non-Pythonic types and workflows).

When a function is overloaded only by its `const`-ness, choose to bind the
mutable overload, not the `const` overload.

We do this because `pybind11` evaluates overloads in the order they are bound,
and is not possible for a user to "reach" any overloads that can only be
disambiguated by const-ness.

Examples of functions to bind and not bind:

<!--
WARNING: Adding {.cc} for syntax coloring seems to cause Doxygen to break
(#15439).
-->

```
// N.B. The two free functions below aren't necessarily great in terms of coding
// and the GSG; however, we use them for illustrative purposes.

// BIND: Should be exposed.
void MyFunction(MyClass* mutable_value);
// DO NOT BIND: Identical to above mutable overload, may constrain user.
void MyFunction(const MyClass& value);

// Disambiguating an accessor solely based on `const`-ness of `this`. In
// general, you may want to avoid this.
class MyOtherClassDispreferred {
 public:
  ...
  // BIND: Even though more costly, this ensure the user has access to the
  // "maximum" amount of functionality.
  MyClass& value();

  // DO NOT BIND: Identical to above mutable overload.
  const MyClass& value() const;
};

class MyOtherClassPreferred {
 public:
  ...
  // BIND: Unambiguous.
  MyClass& mutable_value();

  // BIND: Unambiguous.
  const MyValue& value() const;
};
```

### Public C++ API Considerations for Function and Method Templates

The motivation behind this section can be found under the
"C++ Function and Method Template Instantiations in Python" section in
`doc/python_bindings.rst`.

In general, Drake uses techniques like parameter packs and type erasure to
create sugar functions. These functions map their inputs to parameters of some
concrete, under-the-hood method that actually does the work, and is devoid of
such tricks. To facilitate python bindings, this underlying function should
also be exposed in the public API.

As an example for parameter packs,
`MultibodyPlant<T>::AddJoint<JointType, Args...>(...)`
([code permalink](https://git.io/JfqhI))
is a C++ sugar method
that uses parameter packs and ultimately passes the result to
`MultibodyPlant<T>::AddJoint<JointType>(unique_ptr<JointType>)`
([code permalink](https://git.io/JfqhU)), and only the
`unique_ptr` function is bound ([code permalink](https://git.io/Jfqie)):

```
using Class = MultibodyPlant<T>;
...
    .def("AddJoint",
        [](Class* self, std::unique_ptr<Joint<T>> joint) -> auto& {
          return self->AddJoint(std::move(joint));
        },
        py::arg("joint"), py_rvp::reference_internal, cls_doc.AddJoint.doc_1args)
...
```

As an example for parameter packs,
`GeometryProperties::AddProperty<ValueType>`
([code permalink](https://git.io/JfqhL)) is a C++ sugar method that uses
type erasure and ultimately passes the result to
`GeometryProperties::AddPropertyAbstract`
([code permalink](https://git.io/Jfqhm)), and only the `AddPropertyAbstract`
flavor is used in the bindings, but in such a way that it is similar to the C++
API for `AddProperty` ([code permalink](https://git.io/JfqiT)):

```
using Class = GeometryProperties;
py::handle abstract_value_cls =
    py::module::import("pydrake.systems.framework").attr("AbstractValue");
...
    .def("AddProperty",
        [abstract_value_cls](Class* self, const std::string& group_name,
            const std::string& name, py::object value) {
          py::object abstract = abstract_value_cls.attr("Make")(value);
          self->AddPropertyAbstract(
              group_name, name, abstract.cast<const AbstractValue&>());
        },
        py::arg("group_name"), py::arg("name"), py::arg("value"),
        cls_doc.AddProperty.doc)
...
```

### Matrix-multiplication-like Methods

For objects that may be represented by matrices or vectors (e.g.
RigidTransform, RotationMatrix), the `*` operator (via `__mul__`) should *not*
be bound because the `*` operator in NumPy implies elemnt-wise multiplication
for arrays.

For simplicity, we instead bind the explicitly named `.multiply()` method, and
alias the `__matmul__` operator `@` to this function.

### Clone Methods

If you wish to bind a `Clone()` method, please use `DefClone()` so that
`__copy__()` and `__deepcopy__()` will also be defined. (It is simplest to do
these things together.)

@anchor PydrakeReturnVectorsOrMatrices
#### Returning Vectors or Matrices

Certain bound methods, like `RigidTransform.multiply()`, will have overloads
that can multiply and return (a) other `RigidTransform` instances, (b) vectors,
or (c) matrices (representing a list of vectors).

In the cases of (a) and (c), `pybind11` provides sufficient mechanisms to
provide an unambiguous output return type. However, for (b), `pybind11` will
return `ndarray` with shape `(3,)`. This can cause an issue when users pass
a vector of shape `(3, 1)` as input. Nominally, pybind11 will return a `(3,)`
array, but the user may expect `(3, 1)` as an output. To accommodate this, you
should use the drake::pydrake::WrapToMatchInputShape function.

@sa https://github.com/RobotLocomotion/drake/issues/13885

## Python Subclassing of C++ Classes

In general, minimize the amount in which users may subclass C++ classes in
Python. When you do wish to do this, ensure that you use a trampoline class
in `pybind`, and ensure that the trampoline class inherits from the
`py::wrapper<>` class specific to our fork of `pybind`. This ensures that no
slicing happens with the subclassed instances.

@anchor PydrakeBazelDebug
# Interactive Debugging with Bazel

If you are debugging a unitest, first try running the test with `--trace=user`
to see where the code is failing. This should cover most cases where you need
to debug C++ bits. Example:

    bazel run //bindings/pydrake/systems:py/lifetime_test -- --trace=user

If you need to debug further while using Bazel, it is suggested to use
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
