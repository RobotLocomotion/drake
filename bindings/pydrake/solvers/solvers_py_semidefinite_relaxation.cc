#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/semidefinite_relaxation.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversSemidefiniteRelaxation(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"), doc.MakeSemidefiniteRelaxation.doc_1args);
  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&,
          const std::vector<symbolic::Variables>&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"), py::arg("variable_groups"),
      doc.MakeSemidefiniteRelaxation.doc_2args);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
