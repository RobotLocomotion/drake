#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/branch_and_bound.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversBranchAndBound(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  {
    using Class = MixedIntegerBranchAndBound;
    constexpr auto& cls_doc = doc.MixedIntegerBranchAndBound;
    py::class_<Class> bnb_cls(m, "MixedIntegerBranchAndBound", cls_doc.doc);

    {
      using Nested = MixedIntegerBranchAndBound::Options;
      constexpr auto& options_doc = cls_doc.Options;
      py::class_<Nested> options_cls(bnb_cls, "Options", options_doc.doc);
      options_cls.def(ParamInit<Nested>());
      DefAttributesUsingSerialize(&options_cls, options_doc);
      DefReprUsingSerialize(&options_cls);
      DefCopyAndDeepCopy(&options_cls);
    }

    bnb_cls
        .def(py::init<const MathematicalProgram&, const SolverId&,
                 MixedIntegerBranchAndBound::Options>(),
            py::arg("prog"), py::arg("solver_id"),
            py::arg("options") = MixedIntegerBranchAndBound::Options{},
            cls_doc.ctor.doc)
        .def("Solve", &Class::Solve, cls_doc.Solve.doc)
        .def("GetOptimalCost", &Class::GetOptimalCost,
            cls_doc.GetOptimalCost.doc)
        .def(
            "GetSubOptimalCost",
            [](const Class& self, int nth_suboptimal_cost) {
              return self.GetSubOptimalCost(nth_suboptimal_cost);
            },
            py::arg("nth_suboptimal_cost"), cls_doc.GetSubOptimalCost.doc)
        .def(
            "GetSolution",
            [](const Class& self, const symbolic::Variable& mip_var,
                int nth_best_solution) {
              return self.GetSolution(mip_var, nth_best_solution);
            },
            py::arg("mip_var"), py::arg("nth_best_solution") = 0,
            cls_doc.GetSolution.doc_2args_mip_var_nth_best_solution)
        .def(
            "GetSolution",
            [](const Class& self, const VectorXDecisionVariable& mip_vars,
                int nth_best_solution) {
              return self.GetSolution(mip_vars, nth_best_solution);
            },
            py::arg("mip_vars"), py::arg("nth_best_solution") = 0,
            cls_doc.GetSolution.doc_2args_constEigenMatrixBase_int)
        .def(
            "GetSolution",
            [](const Class& self, const MatrixXDecisionVariable& mip_vars,
                int nth_best_solution) {
              return self.GetSolution(mip_vars, nth_best_solution);
            },
            py::arg("mip_vars"), py::arg("nth_best_solution") = 0,
            cls_doc.GetSolution.doc_2args_constEigenMatrixBase_int);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
