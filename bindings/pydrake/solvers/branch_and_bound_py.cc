#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/branch_and_bound.h"

namespace drake {
namespace pydrake {
PYBIND11_MODULE(branch_and_bound, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "branch_and_bound bindings for MathematicalProgram";

  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<MixedIntegerBranchAndBound>(
      m, "MixedIntegerBranchAndBound", doc.MixedIntegerBranchAndBound.doc)
      .def(py::init(
               [](const MathematicalProgram& prog, const SolverId& solver_id) {
                 return std::unique_ptr<MixedIntegerBranchAndBound>(
                     new MixedIntegerBranchAndBound(prog, solver_id));
               }),
          py::arg("prog"), py::arg("solver_id"),
          doc.MixedIntegerBranchAndBound.ctor.doc)
      .def("Solve", &MixedIntegerBranchAndBound::Solve,
          doc.MixedIntegerBranchAndBound.Solve.doc)
      .def("GetOptimalCost", &MixedIntegerBranchAndBound::GetOptimalCost,
          doc.MixedIntegerBranchAndBound.GetOptimalCost.doc)
      .def("GetSubOptimalCost",
          [](const MixedIntegerBranchAndBound& self, int nth_suboptimal_cost) {
            return self.GetSubOptimalCost(nth_suboptimal_cost);
          },
          py::arg("nth_suboptimal_cost"),
          doc.MixedIntegerBranchAndBound.GetSubOptimalCost.doc)
      .def("GetSolution",
          [](const MixedIntegerBranchAndBound& self,
              const symbolic::Variable& mip_var, int nth_best_solution) {
            return self.GetSolution(mip_var, nth_best_solution);
          },
          py::arg("mip_var"), py::arg("nth_best_solution") = 0,
          doc.MixedIntegerBranchAndBound.GetSolution
              .doc_2args_mip_var_nth_best_solution)
      .def("GetSolution",
          [](const MixedIntegerBranchAndBound& self,
              const VectorXDecisionVariable& mip_vars, int nth_best_solution) {
            return self.GetSolution(mip_vars, nth_best_solution);
          },
          py::arg("mip_var"), py::arg("nth_best_solution") = 0,
          doc.MixedIntegerBranchAndBound.GetSolution
              .doc_2args_constEigenMatrixBase_int)
      .def("GetSolution",
          [](const MixedIntegerBranchAndBound& self,
              const MatrixXDecisionVariable& mip_vars, int nth_best_solution) {
            return self.GetSolution(mip_vars, nth_best_solution);
          },
          py::arg("mip_var"), py::arg("nth_best_solution") = 0,
          doc.MixedIntegerBranchAndBound.GetSolution
              .doc_2args_constEigenMatrixBase_int);
}

}  // namespace pydrake
}  // namespace drake
