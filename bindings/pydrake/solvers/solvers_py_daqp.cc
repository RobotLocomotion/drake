#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/daqp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversDaqp(py::module_ m) {
  using namespace drake::solvers;  // NOLINT(build/namespaces)

  class_<DaqpSolver, SolverInterface>(m, "DaqpSolver")
      .def(py::init<>())
      .def_static("id", &DaqpSolver::id);

  class_<DaqpSolverDetails>(m, "DaqpSolverDetails")
      .def_ro("exitflag", &DaqpSolverDetails::exitflag)
      .def_ro("iterations", &DaqpSolverDetails::iterations)
      .def_ro("setup_time", &DaqpSolverDetails::setup_time)
      .def_ro("solve_time", &DaqpSolverDetails::solve_time)
      .def_ro("multipliers", &DaqpSolverDetails::multipliers);
  AddValueInstantiation<DaqpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
