#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

// Defined in `framework_py_systems.cc`.
void DefineFrameworkPySystems(py::module m);
// Defined in `framework_py_semantics.cc`.
void DefineFrameworkPySemantics(py::module m);
// Defined in `framework_py_values.cc`.
void DefineFrameworkPyValues(py::module m);

PYBIND11_MODULE(framework, m) {
  m.doc() = "Bindings for the core Systems framework.";

  // Incorporate definitinos as pieces (to speed up compilation).
  DefineFrameworkPySystems(m);
  DefineFrameworkPySemantics(m);
  DefineFrameworkPyValues(m);
}

}  // namespace pydrake
}  // namespace drake
