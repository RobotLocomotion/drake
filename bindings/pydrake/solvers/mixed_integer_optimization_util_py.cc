#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace pydrake {
PYBIND11_MODULE(mixed_integer_optimization_util, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "mixed integer optimization utils bindings";

  py::module::import("pydrake.solvers.mathematicalprogram");

  m.def(
      "AddLogarithmicSos2Constraint",
      [](MathematicalProgram* prog,
          const VectorX<symbolic::Expression>& lambdas,
          const std::string& binary_variable_name) {
        return AddLogarithmicSos2Constraint(
            prog, lambdas, binary_variable_name);
      },
      py::arg("prog"), py::arg("lambdas"),
      py::arg("binary_variable_name") = "y",
      doc.AddLogarithmicSos2Constraint.doc_3args_prog_lambda_y);

  m.def("AddSos2Constraint", &AddSos2Constraint, py::arg("prog"),
      py::arg("lambdas"), py::arg("y"), doc.AddSos2Constraint.doc);

  m.def(
      "AddLogarithmicSos1Constraint",
      [](MathematicalProgram* prog, int num_sections) {
        return AddLogarithmicSos1Constraint(prog, num_sections);
      },
      py::arg("prog"), py::arg("num_lambda"),
      doc.AddLogarithmicSos1Constraint.doc_2args);

  {
    py::enum_<IntervalBinning>(m, "IntervalBinning", doc.IntervalBinning.doc)
        .value("kLogarithmic", IntervalBinning::kLogarithmic)
        .value("kLinear", IntervalBinning::kLinear);
  }
}
}  // namespace pydrake
}  // namespace drake
