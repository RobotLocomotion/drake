#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {

using symbolic::Variable;

namespace pydrake {

PYBIND11_MODULE(odr_test_module, m) {
  m.doc() = "Test ODR using Variable.";

  m.def("new_variable",
      [](const std::string& name) { return new Variable(name); });
}

}  // namespace pydrake
}  // namespace drake
