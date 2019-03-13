/// @file
/// Alias and bind deprecated symbols from old `//multibody/multibody_tree`
/// package.

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

void ForwardSymbols(py::module from, py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(py::str("from {} import *").format(from.attr("__name__")),
      py::globals(), vars);

  py::str deprecation = py::str(
      "``{}`` is deprecated and will be removed on or around "
      "2019-05-01. Please use ``{}`` instead")
                            .format(m.attr("__name__"), from.attr("__name__"));
  WarnDeprecated(deprecation);
  m.doc() = py::str("Warning:\n    {}").format(deprecation);
}

// Bind deprecated symbols.
void init_module(py::module m) {
  ForwardSymbols(py::module::import("pydrake.multibody.tree"), m);
}

void init_math(py::module m) {
  ForwardSymbols(py::module::import("pydrake.multibody.math"), m);
}

void init_multibody_plant(py::module m) {
  ForwardSymbols(py::module::import("pydrake.multibody.plant"), m);
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.multibody.multibody_tree import *\n"
      "from pydrake.multibody.multibody_tree.math import *\n"
      "from pydrake.multibody.multibody_tree.multibody_plant import *\n",
      py::globals(), vars);
  // N.B. Deprecation will have been emitted by `multibody_tree` module already.
  m.doc() =
      "Warning:\n   ``pydrake.multibody.multibody_tree.all`` is deprecated "
      "and will be removed on or around 2019-05-01.";
}

}  // namespace

PYBIND11_MODULE(multibody_tree, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  init_module(m);
  init_math(m.def_submodule("math"));
  init_multibody_plant(m.def_submodule("multibody_plant"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
