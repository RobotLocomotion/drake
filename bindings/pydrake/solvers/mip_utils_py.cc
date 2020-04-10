#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace pydrake {
PYBIND11_MODULE(mip_utils, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "mixed integer optimization utils bindings";

  py::module::import("pydrake.solvers.mathematicalprogram");

  m.def("AddLogarithmicSos1Condition",
      [](MathematicalProgram* prog, const VectorX<symbolic::Expression>& lambda,
          const VectorX<symbolic::Variable>& y, const Eigen::MatrixXi& codes) {
        return AddLogarithmicSos1Constraint(prog, lambda, binary_variable_name);
      },
      py::arg("prog"), py::arg("lambda"), py::arg("binary_variable_name") = "y",
      doc.AddLogarithmicSos1Constraint.doc);
}
}  // namespace pydrake
}  // namespace drake
