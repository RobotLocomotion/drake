#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace py = pybind11;

using drake::symbolic::Variable;

PYBIND11_MODULE(odr_test_module, m) {
  m.doc() = "Test ODR using Variable.";

  m.def("new_variable", [](const std::string& name) {
    return new Variable(name);
  });
}
