#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/dual_convex_program.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversDualConvexProgram(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.def(
      "CreateDualConvexProgram",
      [](const MathematicalProgram& prog) {
        std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
            constraint_to_dual_variable_map;
        auto dual =
            CreateDualConvexProgram(prog, &constraint_to_dual_variable_map);
        return std::make_pair(std::move(dual), constraint_to_dual_variable_map);
      },
      py::arg("prog"), doc.CreateDualConvexProgram.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
