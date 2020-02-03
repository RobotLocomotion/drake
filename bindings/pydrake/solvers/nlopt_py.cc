#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/nlopt_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(nlopt, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

    m.doc() = R"""(
Python bindings for the MathematicalProgram Nlopt solver.

For original API documentation, please refer to the
`solvers module <https://drake.mit.edu/doxygen_cxx/group__solvers.html>`_
in the Doxygen C++ documentation.
)""";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.systems.framework");

  py::class_<NloptSolver, SolverInterface>(
      m, "NloptSolver", doc.NloptSolver.doc)
      .def(py::init<>(), doc.NloptSolver.ctor.doc);

  py::class_<NloptSolverDetails>(
      m, "NloptSolverDetails", doc.NloptSolverDetails.doc)
      .def_readwrite("status", &NloptSolverDetails::status,
          doc.NloptSolverDetails.status.doc);
  AddValueInstantiation<NloptSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
