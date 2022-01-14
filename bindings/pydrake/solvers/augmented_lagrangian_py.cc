#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/augmented_lagrangian.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(augmented_lagrangian, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Augmented Lagrangian solver bindings for MathematicalProgram";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.autodiffutils");

  m.def(
       "EvalAugmentedLagrangian",
       [](const MathematicalProgram& prog,
           const Eigen::Ref<const Eigen::VectorXd>& x,
           const Eigen::VectorXd& lambda_val, double mu,
           bool include_x_bounds) {
         Eigen::VectorXd constraint_residue;
         double cost;
         const double al_val = EvalAugmentedLagrangian<double>(prog, x,
             lambda_val, mu, include_x_bounds, &constraint_residue, &cost);
         return std::make_tuple(al_val, constraint_residue, cost);
       },
       py::arg("prog"), py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
       py::arg("include_x_bounds"), doc.EvalAugmentedLagrangian.doc)
      .def(
          "EvalAugmentedLagrangian",
          [](const MathematicalProgram& prog,
              const Eigen::Ref<const VectorX<AutoDiffXd>>& x,
              const Eigen::VectorXd& lambda_val, double mu,
              bool include_x_bounds) {
            VectorX<AutoDiffXd> constraint_residue;
            AutoDiffXd cost;
            const AutoDiffXd al_val =
                EvalAugmentedLagrangian<AutoDiffXd>(prog, x, lambda_val, mu,
                    include_x_bounds, &constraint_residue, &cost);
            return std::make_tuple(al_val, constraint_residue, cost);
          },
          py::arg("prog"), py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
          py::arg("include_x_bounds"), doc.EvalAugmentedLagrangian.doc)
      .def("GetLagrangianSizeForAl", &GetLagrangianSizeForAl, py::arg("prog"),
          py::arg("include_x_bounds"), doc.GetLagrangianSizeForAl.doc)
      .def("IsEqualityForAl", &IsEqualityForAl, py::arg("prog"),
          py::arg("include_x_bounds"), doc.IsEqualityForAl.doc);
}
}  // namespace pydrake
}  // namespace drake
