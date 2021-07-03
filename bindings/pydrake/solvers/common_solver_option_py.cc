#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/common_solver_option.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(common_solver_option, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "common_solver_option bindings for MathematicalProgram";

  py::enum_<CommonSolverOption>(
      m, "CommonSolverOption", doc.CommonSolverOption.doc)
      .value("kPrintFileName", CommonSolverOption::kPrintFileName,
          doc.CommonSolverOption.kPrintFileName.doc)
      .value("kPrintToConsole", CommonSolverOption::kPrintToConsole,
          doc.CommonSolverOption.kPrintToConsole.doc);
}
}  // namespace pydrake
}  // namespace drake
